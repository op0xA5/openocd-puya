// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   CMSIS-DAP net_tcp backend                                             *
 *                                                                         *
 *   Connects to a DAP-over-TCP server which bridges CMSIS-DAP USB         *
 *   devices. See doc/cmsis-dap-net-tcp-protocol.md for the protocol.      *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/system.h>
#include <helper/log.h>
#include <helper/types.h>
#include <helper/replacements.h>

#include <string.h>
#include <errno.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#endif

#include "cmsis_dap.h"

/* Protocol definitions, see doc/cmsis-dap-net-tcp-protocol.md */
#define NET_TCP_MAGIC		0xfffe

#define NET_TCP_CMD_LIST	0x0001
#define NET_TCP_CMD_OPEN	0x0002
#define NET_TCP_CMD_CLOSE	0x0003
#define NET_TCP_CMD_DAP_DATA	0x0004

#define NET_TCP_STATUS_REQUEST	0x0000
#define NET_TCP_STATUS_OK	0x0001
#define NET_TCP_STATUS_PARTIAL	0x0002
#define NET_TCP_STATUS_ERROR	0x0100

#define NET_TCP_HDR_LEN		8
#define NET_TCP_DEV_INFO_LEN	0x88
#define NET_TCP_MAX_DEVICES	32

#define NET_TCP_DEFAULT_HOST	"127.0.0.1"
#define NET_TCP_DEFAULT_PORT	1234

/* Timeout for handshake stage messages */
#define NET_TCP_HANDSHAKE_TIMEOUT_MS	5000

struct net_tcp_dev_info {
	uint8_t raw[NET_TCP_DEV_INFO_LEN];
	uint16_t vid;
	uint16_t pid;
	uint16_t packet_size;
	char product[64];
	char serial[64];
};

struct cmsis_dap_backend_data {
	int fd;
};

static char net_tcp_host[256] = NET_TCP_DEFAULT_HOST;
static uint16_t net_tcp_port = NET_TCP_DEFAULT_PORT;

static void net_tcp_dev_info_parse(struct net_tcp_dev_info *info, const uint8_t *raw)
{
	memcpy(info->raw, raw, NET_TCP_DEV_INFO_LEN);
	info->vid = be_to_h_u16(raw + 2);
	info->pid = be_to_h_u16(raw + 4);
	info->packet_size = be_to_h_u16(raw + 6);
	memcpy(info->product, raw + 8, sizeof(info->product));
	memcpy(info->serial, raw + 72, sizeof(info->serial));
	/* make sure the strings are terminated even if the server
	 * did not leave room for a NUL */
	info->product[sizeof(info->product) - 1] = '\0';
	info->serial[sizeof(info->serial) - 1] = '\0';
}

/* Wait until the socket is readable, returns 1 on readable,
 * 0 on timeout, ERROR_FAIL on error */
static int net_tcp_wait_readable(int fd, int timeout_ms)
{
	fd_set rfds;
	struct timeval tv;
	int retval;

#ifdef _WIN32
	if (fd >= (int)FD_SETSIZE)
		return ERROR_FAIL;
#endif

	FD_ZERO(&rfds);
	FD_SET((unsigned int)fd, &rfds);
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	do {
		retval = select(fd + 1, &rfds, NULL, NULL, &tv);
	} while (retval < 0 && errno == EINTR);

	if (retval < 0) {
		LOG_ERROR("net_tcp: select failed: %s", strerror(errno));
		return ERROR_FAIL;
	}

	return retval > 0 ? 1 : 0;
}

static int net_tcp_send_all(int fd, const uint8_t *buf, size_t len)
{
#ifdef MSG_NOSIGNAL
	const int flags = MSG_NOSIGNAL;
#else
	const int flags = 0;
#endif

	while (len) {
		int written = send(fd, (const char *)buf, len, flags);
		if (written < 0) {
			if (errno == EINTR)
				continue;
			LOG_ERROR("net_tcp: send failed: %s", strerror(errno));
			return ERROR_FAIL;
		}
		if (written == 0) {
			LOG_ERROR("net_tcp: connection closed while sending");
			return ERROR_FAIL;
		}
		buf += written;
		len -= written;
	}

	return ERROR_OK;
}

/* Receive exactly len bytes, waiting at most timeout_ms for each chunk */
static int net_tcp_recv_all(int fd, uint8_t *buf, size_t len, int timeout_ms)
{
	while (len) {
		int retval = net_tcp_wait_readable(fd, timeout_ms);
		if (retval <= 0)
			return retval == 0 ? ERROR_TIMEOUT_REACHED : ERROR_FAIL;

		int received = recv(fd, (char *)buf, len, 0);
		if (received < 0) {
			if (errno == EINTR)
				continue;
			LOG_ERROR("net_tcp: recv failed: %s", strerror(errno));
			return ERROR_FAIL;
		}
		if (received == 0) {
			LOG_ERROR("net_tcp: connection closed by peer");
			return ERROR_FAIL;
		}
		buf += received;
		len -= received;
	}

	return ERROR_OK;
}

static int net_tcp_send_msg(int fd, uint16_t cmd, uint16_t status,
		const uint8_t *payload, uint16_t len)
{
	uint8_t hdr[NET_TCP_HDR_LEN];

	h_u16_to_be(hdr, NET_TCP_MAGIC);
	h_u16_to_be(hdr + 2, cmd);
	h_u16_to_be(hdr + 4, status);
	h_u16_to_be(hdr + 6, len);

	int retval = net_tcp_send_all(fd, hdr, sizeof(hdr));
	if (retval != ERROR_OK)
		return retval;

	if (len)
		retval = net_tcp_send_all(fd, payload, len);

	return retval;
}

/* Receive one protocol message.
 * payload may be NULL if no payload is expected. Oversized or unexpected
 * payload bytes are read and discarded.
 * Returns ERROR_OK, ERROR_TIMEOUT_REACHED or ERROR_FAIL. */
static int net_tcp_recv_msg(int fd, int timeout_ms, uint16_t expected_cmd,
		uint16_t *status, uint8_t *payload, size_t payload_cap, uint16_t *payload_len)
{
	uint8_t hdr[NET_TCP_HDR_LEN];

	int retval = net_tcp_wait_readable(fd, timeout_ms);
	if (retval == 0)
		return ERROR_TIMEOUT_REACHED;
	if (retval < 0)
		return ERROR_FAIL;

	retval = net_tcp_recv_all(fd, hdr, sizeof(hdr), timeout_ms);
	if (retval != ERROR_OK)
		return retval;

	uint16_t magic = be_to_h_u16(hdr);
	uint16_t cmd = be_to_h_u16(hdr + 2);
	uint16_t len = be_to_h_u16(hdr + 6);

	if (magic != NET_TCP_MAGIC) {
		LOG_ERROR("net_tcp: bad magic 0x%04" PRIx16, magic);
		return ERROR_FAIL;
	}
	if (cmd != expected_cmd) {
		LOG_ERROR("net_tcp: unexpected command 0x%04" PRIx16
			" (expected 0x%04" PRIx16 ")", cmd, expected_cmd);
		return ERROR_FAIL;
	}

	*status = be_to_h_u16(hdr + 4);
	*payload_len = len;

	size_t keep = MIN(len, payload_cap);
	if (payload && keep) {
		retval = net_tcp_recv_all(fd, payload, keep, timeout_ms);
		if (retval != ERROR_OK)
			return retval;
	}

	/* discard any payload bytes that do not fit the buffer */
	uint8_t discard[64];
	for (size_t rest = len - keep; rest; ) {
		size_t chunk = MIN(rest, sizeof(discard));
		retval = net_tcp_recv_all(fd, discard, chunk, timeout_ms);
		if (retval != ERROR_OK)
			return retval;
		rest -= chunk;
	}

	return ERROR_OK;
}

/* Log the text carried by an error response */
static void net_tcp_log_error(uint16_t status, const uint8_t *payload, uint16_t len)
{
	char msg[257];

	size_t n = MIN(len, sizeof(msg) - 1);
	if (payload && n)
		memcpy(msg, payload, n);
	msg[n] = '\0';

	LOG_ERROR("net_tcp: server error 0x%04" PRIx16 ": %s", status, msg);
}

static int net_tcp_connect(const char *host, uint16_t port)
{
	struct addrinfo hints = {
		.ai_family = AF_UNSPEC,
		.ai_socktype = SOCK_STREAM,
		.ai_protocol = IPPROTO_TCP,
	};
	struct addrinfo *result, *rp;
	char port_str[8];
	int fd = -1;

	snprintf(port_str, sizeof(port_str), "%" PRIu16, port);

	int err = getaddrinfo(host, port_str, &hints, &result);
	if (err) {
		LOG_ERROR("net_tcp: cannot resolve %s: %s", host, gai_strerror(err));
		return -1;
	}

	for (rp = result; rp; rp = rp->ai_next) {
		fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (fd < 0)
			continue;
		if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0)
			break;
#ifdef _WIN32
		closesocket(fd);
#else
		close(fd);
#endif
		fd = -1;
	}

	freeaddrinfo(result);

	if (fd < 0) {
		LOG_ERROR("net_tcp: cannot connect to %s:%" PRIu16, host, port);
		return -1;
	}

	int one = 1;
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (void *)&one, sizeof(one));
	setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, (void *)&one, sizeof(one));
#ifdef SO_NOSIGPIPE
	/* avoid SIGPIPE on systems without MSG_NOSIGNAL */
	setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, (void *)&one, sizeof(one));
#endif

	return fd;
}

/* Fetch the device list from the server.
 * Returns the number of devices, or a negative error code. */
static int net_tcp_list_devices(int fd, struct net_tcp_dev_info *devs, int max_devs)
{
	int retval = net_tcp_send_msg(fd, NET_TCP_CMD_LIST,
			NET_TCP_STATUS_REQUEST, NULL, 0);
	if (retval != ERROR_OK)
		return retval;

	int count = 0;
	for (;;) {
		uint8_t buf[NET_TCP_DEV_INFO_LEN];
		uint16_t status, len;

		retval = net_tcp_recv_msg(fd, NET_TCP_HANDSHAKE_TIMEOUT_MS,
				NET_TCP_CMD_LIST, &status, buf, sizeof(buf), &len);
		if (retval != ERROR_OK)
			return retval;

		if (status >= NET_TCP_STATUS_ERROR) {
			net_tcp_log_error(status, buf, len);
			return ERROR_FAIL;
		}

		if (len == NET_TCP_DEV_INFO_LEN) {
			if (count >= max_devs) {
				LOG_WARNING("net_tcp: too many devices, ignoring the rest");
			} else {
				net_tcp_dev_info_parse(&devs[count], buf);
				LOG_DEBUG("net_tcp: device %d: %04x:%04x \"%s\" serial \"%s\" packet_size %" PRIu16,
					count + 1, devs[count].vid, devs[count].pid,
					devs[count].product, devs[count].serial,
					devs[count].packet_size);
				count++;
			}
		} else if (len != 0) {
			LOG_ERROR("net_tcp: bad device info length %" PRIu16, len);
			return ERROR_FAIL;
		}

		if (status == NET_TCP_STATUS_OK)
			break;
		if (status != NET_TCP_STATUS_PARTIAL) {
			LOG_ERROR("net_tcp: bad list response status 0x%04" PRIx16, status);
			return ERROR_FAIL;
		}
	}

	return count;
}

static bool net_tcp_dev_info_match(const struct net_tcp_dev_info *info,
		uint16_t vids[], uint16_t pids[], const char *serial)
{
	bool id_filter = vids[0] || pids[0];

	if (id_filter) {
		bool id_match = false;
		for (int id = 0; vids[id] || pids[id]; id++) {
			id_match = !vids[id] || info->vid == vids[id];
			id_match &= !pids[id] || info->pid == pids[id];
			if (id_match)
				break;
		}
		if (!id_match)
			return false;
	}

	if (serial && serial[0] && strcmp(info->serial, serial) != 0)
		return false;

	return true;
}

static void net_tcp_socket_close(struct cmsis_dap *dap)
{
	if (!dap->bdata)
		return;

	if (dap->bdata->fd >= 0) {
#ifdef _WIN32
		closesocket(dap->bdata->fd);
#else
		close(dap->bdata->fd);
#endif
	}
	free(dap->bdata);
	dap->bdata = NULL;
}

static int cmsis_dap_net_tcp_open(struct cmsis_dap *dap, uint16_t vids[], uint16_t pids[], const char *serial)
{
#ifdef _WIN32
	static bool wsa_started;
	if (!wsa_started) {
		WSADATA wsa;
		if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
			LOG_ERROR("net_tcp: WSAStartup failed");
			return ERROR_FAIL;
		}
		wsa_started = true;
	}
#endif

	int fd = net_tcp_connect(net_tcp_host, net_tcp_port);
	if (fd < 0)
		return ERROR_FAIL;

	dap->bdata = calloc(1, sizeof(struct cmsis_dap_backend_data));
	if (!dap->bdata) {
		LOG_ERROR("unable to allocate memory");
#ifdef _WIN32
		closesocket(fd);
#else
		close(fd);
#endif
		return ERROR_FAIL;
	}
	dap->bdata->fd = fd;

	struct net_tcp_dev_info devs[NET_TCP_MAX_DEVICES];
	int num_devices = net_tcp_list_devices(fd, devs, NET_TCP_MAX_DEVICES);
	if (num_devices < 0)
		goto init_err;

	/* Build the open request: either echo back the matching list entry,
	 * or send the requested VID/PID/serial as filter */
	uint8_t open_req[NET_TCP_DEV_INFO_LEN] = { 0 };
	const struct net_tcp_dev_info *match = NULL;

	for (int i = 0; i < num_devices; i++) {
		if (net_tcp_dev_info_match(&devs[i], vids, pids, serial)) {
			match = &devs[i];
			break;
		}
	}

	if (match) {
		memcpy(open_req, match->raw, sizeof(open_req));
		open_req[6] = 0;	/* packet_size is not used by the server */
		open_req[7] = 0;
	} else {
		/* No list or no match: let the server pick the first device
		 * matching VID/PID (first pair only) and serial string */
		if (vids[0])
			h_u16_to_be(open_req + 2, vids[0]);
		if (pids[0])
			h_u16_to_be(open_req + 4, pids[0]);
		if (serial && serial[0]) {
			size_t n = MIN(strlen(serial), 63);
			memcpy(open_req + 72, serial, n);
		}
	}

	int retval = net_tcp_send_msg(fd, NET_TCP_CMD_OPEN,
			NET_TCP_STATUS_REQUEST, open_req, sizeof(open_req));
	if (retval != ERROR_OK)
		goto init_err;

	uint8_t resp[NET_TCP_DEV_INFO_LEN];
	uint16_t status, len;
	retval = net_tcp_recv_msg(fd, NET_TCP_HANDSHAKE_TIMEOUT_MS,
			NET_TCP_CMD_OPEN, &status, resp, sizeof(resp), &len);
	if (retval != ERROR_OK)
		goto init_err;

	if (status >= NET_TCP_STATUS_ERROR) {
		net_tcp_log_error(status, resp, len);
		goto init_err;
	}

	if (status != NET_TCP_STATUS_OK || len != NET_TCP_DEV_INFO_LEN) {
		LOG_ERROR("net_tcp: bad open response (status 0x%04" PRIx16
			" len %" PRIu16 ")", status, len);
		goto init_err;
	}

	struct net_tcp_dev_info opened;
	net_tcp_dev_info_parse(&opened, resp);
	if (!opened.packet_size) {
		LOG_ERROR("net_tcp: server reported packet_size 0");
		goto init_err;
	}

	LOG_INFO("net_tcp: opened %04x:%04x \"%s\" serial \"%s\" at %s:%" PRIu16
		", packet_size %" PRIu16,
		opened.vid, opened.pid, opened.product, opened.serial,
		net_tcp_host, net_tcp_port, opened.packet_size);

	dap->packet_size = opened.packet_size;
	dap->packet_buffer_size = opened.packet_size;
	dap->packet_buffer = malloc(dap->packet_buffer_size);
	if (!dap->packet_buffer) {
		LOG_ERROR("unable to allocate memory");
		goto init_err;
	}

	dap->command = dap->packet_buffer;
	dap->response = dap->packet_buffer;

	return ERROR_OK;

init_err:
	net_tcp_socket_close(dap);
	return ERROR_FAIL;
}

static void cmsis_dap_net_tcp_close(struct cmsis_dap *dap)
{
	if (dap->bdata && dap->bdata->fd >= 0) {
		uint16_t status, len;

		/* Ask the server to close the device, but do not insist */
		if (net_tcp_send_msg(dap->bdata->fd, NET_TCP_CMD_CLOSE,
				NET_TCP_STATUS_REQUEST, NULL, 0) == ERROR_OK)
			net_tcp_recv_msg(dap->bdata->fd, 1000, NET_TCP_CMD_CLOSE,
					&status, NULL, 0, &len);
	}

	net_tcp_socket_close(dap);
	free(dap->packet_buffer);
	dap->packet_buffer = NULL;
}

static int cmsis_dap_net_tcp_read(struct cmsis_dap *dap, int timeout_ms)
{
	uint16_t status, len;

	int retval = net_tcp_recv_msg(dap->bdata->fd, timeout_ms,
			NET_TCP_CMD_DAP_DATA, &status,
			dap->packet_buffer, dap->packet_buffer_size, &len);
	if (retval != ERROR_OK)
		return retval;

	if (status >= NET_TCP_STATUS_ERROR) {
		net_tcp_log_error(status, dap->packet_buffer, len);
		return ERROR_FAIL;
	}

	if (status != NET_TCP_STATUS_OK) {
		LOG_ERROR("net_tcp: bad data response status 0x%04" PRIx16, status);
		return ERROR_FAIL;
	}

	memset(&dap->packet_buffer[len], 0, dap->packet_buffer_size - len);

	return len;
}

static int cmsis_dap_net_tcp_write(struct cmsis_dap *dap, int txlen, int timeout_ms)
{
	int retval = net_tcp_send_msg(dap->bdata->fd, NET_TCP_CMD_DAP_DATA,
			NET_TCP_STATUS_REQUEST, dap->packet_buffer, txlen);
	if (retval != ERROR_OK)
		return retval;

	return txlen;
}

static int cmsis_dap_net_tcp_alloc(struct cmsis_dap *dap, unsigned int pkt_sz)
{
	uint8_t *buf = malloc(pkt_sz);
	if (!buf) {
		LOG_ERROR("unable to allocate CMSIS-DAP packet buffer");
		return ERROR_FAIL;
	}

	dap->packet_buffer = buf;
	dap->packet_size = pkt_sz;
	dap->packet_buffer_size = pkt_sz;

	dap->command = dap->packet_buffer;
	dap->response = dap->packet_buffer;

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_net_tcp_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("expected exactly one argument to cmsis_dap_net tcp <host:port>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	const char *sep = strrchr(CMD_ARGV[0], ':');
	if (!sep || sep == CMD_ARGV[0] || !sep[1]) {
		LOG_ERROR("expected <host:port>, e.g. 127.0.0.1:1234");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	unsigned int port;
	COMMAND_PARSE_NUMBER(uint, sep + 1, port);
	if (port > 65535) {
		LOG_ERROR("invalid port %u", port);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	size_t host_len = sep - CMD_ARGV[0];
	/* strip brackets of an IPv6 literal, e.g. [::1]:1234 */
	if (CMD_ARGV[0][0] == '[' && host_len >= 2 && sep[-1] == ']') {
		CMD_ARGV[0]++;
		host_len -= 2;
	}
	if (host_len >= sizeof(net_tcp_host)) {
		LOG_ERROR("host name too long");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	memcpy(net_tcp_host, CMD_ARGV[0], host_len);
	net_tcp_host[host_len] = '\0';
	net_tcp_port = port;

	LOG_INFO("net_tcp: server set to %s:%" PRIu16, net_tcp_host, net_tcp_port);

	return ERROR_OK;
}

const struct command_registration cmsis_dap_net_subcommand_handlers[] = {
	{
		.name = "tcp",
		.handler = &cmsis_dap_handle_net_tcp_command,
		.mode = COMMAND_CONFIG,
		.help = "set the TCP server address (for net_tcp backend only)",
		.usage = "<host:port>",
	},
	COMMAND_REGISTRATION_DONE
};

const struct cmsis_dap_backend cmsis_dap_net_tcp_backend = {
	.name = "net_tcp",
	.open = cmsis_dap_net_tcp_open,
	.close = cmsis_dap_net_tcp_close,
	.read = cmsis_dap_net_tcp_read,
	.write = cmsis_dap_net_tcp_write,
	.packet_buffer_alloc = cmsis_dap_net_tcp_alloc,
};
