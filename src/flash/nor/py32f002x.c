// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

/* py32f002x register locations */
#define RCC_REG_BASE        0x40021000
#define FLASH_REG_BASE      0x40022000
#define IWDG_REG_BASE       0x40003000

#define FLASH_OPTR_IWDG_SW  (1 << 12)

#define PY32_IWDG_KR        (IWDG_REG_BASE + 0x00)
#define PY32_IWDG_PR        (IWDG_REG_BASE + 0x04)
#define PY32_IWDG_RLR       (IWDG_REG_BASE + 0x08)

#define PY32_RCC_CR         (RCC_REG_BASE + 0x00)
#define PY32_RCC_ICSCR      (RCC_REG_BASE + 0x04)

#define PY32_FLASH_ACR      (FLASH_REG_BASE + 0x00)
#define PY32_FLASH_KEYR     (FLASH_REG_BASE + 0x08)
#define PY32_FLASH_OPTKEYR  (FLASH_REG_BASE + 0x0C)
#define PY32_FLASH_SR       (FLASH_REG_BASE + 0x10)
#define PY32_FLASH_CR       (FLASH_REG_BASE + 0x14)
#define PY32_FLASH_OPTR     (FLASH_REG_BASE + 0x20)
#define PY32_FLASH_SDKR     (FLASH_REG_BASE + 0x24)
#define PY32_FLASH_BTCR     (FLASH_REG_BASE + 0x28)
#define PY32_FLASH_WRPR     (FLASH_REG_BASE + 0x2C)

// #define PY32_FLASH_TS0      (FLASH_REG_BASE + 0x100)
// #define PY32_FLASH_TS1      (FLASH_REG_BASE + 0x104)
// #define PY32_FLASH_TS2P     (FLASH_REG_BASE + 0x108)
// #define PY32_FLASH_TPS3     (FLASH_REG_BASE + 0x10C)
// #define PY32_FLASH_TS3      (FLASH_REG_BASE + 0x110)
// #define PY32_FLASH_PERTPE   (FLASH_REG_BASE + 0x114)
// #define PY32_FLASH_SMERTPE  (FLASH_REG_BASE + 0x118)
// #define PY32_FLASH_PRGTPE   (FLASH_REG_BASE + 0x11C)
// #define PY32_FLASH_PRETPE   (FLASH_REG_BASE + 0x120)

/* option byte location */

// #define PY32_OB_RDP         0x1FFF0080
// #define PY32_OB_USER        0x1FFF0081
// #define PY32_OB_SDK_STRT    0x1FFF0084
// #define PY32_OB_SDK_END     0x1FFF0085
// #define PY32_OB_WRP0        0x1FFF008C
// #define PY32_OB_WRP1        0x1FFF008D

/* FLASH_CR register bits */

#define FLASH_PG            (1 << 0)
#define FLASH_PER           (1 << 1)
#define FLASH_MER           (1 << 2)
#define FLASH_SER           (1 << 11)
#define FLASH_OPTSTRT       (1 << 17)
#define FLASH_PGSTRT        (1 << 19)
#define FLASH_EOPIE         (1 << 24)
#define FLASH_ERRIE         (1 << 25)
#define FLASH_OBL_LAUNCH    (1 << 27)
#define FLASH_OPTLOCK       (1 << 30)
#define FLASH_LOCK          (1 << 31)

/* FLASH_SR register bits */

#define FLASH_EOP           (1 << 0)
#define FLASH_WRPERR        (1 << 4)
#define FLASH_OPTVERR       (1 << 15)
#define FLASH_BSY           (1 << 16)

/* register unlock keys */

#define KEY1                0x45670123
#define KEY2                0xCDEF89AB

#define OPT_KEY1            0x08192A3B
#define OPT_KEY2            0x4C5D6E7F

#define OPTR_DEFAULT        0x4F55B0AA
#define SDKR_DEFAULT        0xFFF4000B
#define BTCR_DEFAULT        0xFFFF0000
#define WRPR_DEFAULT        0xFFC0003F

/* timeout values */

#define FLASH_WRITE_TIMEOUT 100
#define FLASH_ERASE_TIMEOUT 10000

struct py32x_options {
  uint32_t optr;
  uint32_t sdkr;
  uint32_t btcr;
  uint32_t wrpr;
};

struct py32x_flash_bank {
  struct py32x_options option_bytes;
  int psector_size;
  bool probed;

  /* used to access dual flash bank py32xl */
  uint32_t user_bank_size;
};

//static uint32_t gui_rcc_icscr;

static int py32x_init(struct target *target);
static int py32x_uninit(struct target *target);
static int py32x_mass_erase(struct flash_bank *bank);
// static int py32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
//     uint32_t address, uint32_t hwords_count);

/* flash bank py32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(py32x_flash_bank_command)
{
  struct py32x_flash_bank *py32x_info;

  if (CMD_ARGC < 6)
    return ERROR_COMMAND_SYNTAX_ERROR;

  py32x_info = malloc(sizeof(struct py32x_flash_bank));

	bank->driver_priv = py32x_info;
	py32x_info->probed = false;
	py32x_info->user_bank_size = bank->size;

  /* The flash write must be aligned to a 128B boundary */
  bank->write_start_alignment = bank->write_end_alignment = 128;

  return ERROR_OK;
}

static inline int py32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
  struct target *target = bank->target;
  return target_read_u32(target, PY32_FLASH_SR, status);
}

static int py32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
  struct target *target = bank->target;
  uint32_t status;
  int retval = ERROR_OK;

  /* wait for busy to clear */
  for (;;) {
    retval = py32x_get_flash_status(bank, &status);
    target_write_u32(target, PY32_IWDG_KR, 0xAAAA);
    if (retval != ERROR_OK)
      return retval;
    LOG_DEBUG("status: 0x%" PRIx32 "", status);
    if ((status & FLASH_BSY) == 0)
      break;
    if (timeout-- <= 0) {
      LOG_ERROR("timed out waiting for flash");
      return ERROR_FLASH_BUSY;
    }
    alive_sleep(1);
  }

  if (status & FLASH_WRPERR) {
    LOG_ERROR("py32x device protected");
    retval = ERROR_FLASH_PROTECTED;
  }

  if (status & FLASH_OPTVERR) {
    LOG_ERROR("py32x device option and trimming bits loading validity error");
    retval = ERROR_FLASH_OPERATION_FAILED;
  }

  /* Clear but report errors */
  if (status & (FLASH_WRPERR | FLASH_OPTVERR)) {
    /* If this operation fails, we ignore it and report the original
     * retval
     */
    target_write_u32(target, PY32_FLASH_SR,
        FLASH_WRPERR | FLASH_OPTVERR);
  }
  return retval;
}

static int py32x_read_options(struct flash_bank *bank)
{
  struct py32x_flash_bank *py32x_info = bank->driver_priv;
  struct target *target = bank->target;
  int retval;

  /* read user and read protection option bytes, user data option bytes */
  retval = target_read_u32(target, PY32_FLASH_OPTR, &py32x_info->option_bytes.optr);
  if (retval != ERROR_OK)
    return retval;
  
  /* read sdk protection option bytes */
  retval = target_read_u32(target, PY32_FLASH_SDKR, &py32x_info->option_bytes.sdkr);
  if (retval != ERROR_OK)
    return retval;

  /* flash boot control option bytes */
  retval = target_read_u32(target, PY32_FLASH_BTCR, &py32x_info->option_bytes.btcr);
  if (retval != ERROR_OK)
    return retval;

  /* read write protection option bytes */
  retval = target_read_u32(target, PY32_FLASH_WRPR, &py32x_info->option_bytes.wrpr);
  if (retval != ERROR_OK)
    return retval;

  return ERROR_OK;
}

static int py32x_erase_options(struct flash_bank *bank)
{
  struct py32x_flash_bank *py32x_info = bank->driver_priv;
  struct target *target = bank->target;

  /* read current options */
  py32x_read_options(bank);

  /* unlock flash registers */
  int retval = target_write_u32(target, PY32_FLASH_KEYR, KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_KEYR, KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  retval = py32x_init(target);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* unlock option flash registers */
  retval = target_write_u32(target, PY32_FLASH_OPTKEYR, OPT_KEY1);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_OPTKEYR, OPT_KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* erase option bytes */
  retval = target_write_u32(target, PY32_FLASH_OPTR, OPTR_DEFAULT);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_SDKR, SDKR_DEFAULT);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_BTCR, BTCR_DEFAULT);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_WRPR, WRPR_DEFAULT);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_CR, FLASH_OPTSTRT);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, 0x40022080, 0xFFFFFFFF);
  // if (retval != ERROR_OK)
  //   goto flash_lock;

  retval = py32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* clear read protection option byte
   * this will also force a device unlock if set */
  py32x_info->option_bytes.optr = OPTR_DEFAULT;
  py32x_info->option_bytes.sdkr = SDKR_DEFAULT;
  py32x_info->option_bytes.btcr = BTCR_DEFAULT;
  py32x_info->option_bytes.wrpr = WRPR_DEFAULT;

flash_lock:
  py32x_uninit(target);
  target_write_u32(target, PY32_FLASH_CR, FLASH_LOCK);
  return retval;
}

static int py32x_write_options(struct flash_bank *bank)
{
  struct py32x_flash_bank *py32x_info = NULL;
  struct target *target = bank->target;

  py32x_info = bank->driver_priv;

  /* unlock flash registers */
  int retval = target_write_u32(target, PY32_FLASH_KEYR, KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_KEYR, KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* unlock option flash registers */
  retval = target_write_u32(target, PY32_FLASH_OPTKEYR, OPT_KEY1);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_OPTKEYR, OPT_KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  retval = py32x_init(target);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* program option bytes */
  retval = target_write_u32(target, PY32_FLASH_OPTR, py32x_info->option_bytes.optr);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_SDKR, py32x_info->option_bytes.sdkr);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_BTCR, py32x_info->option_bytes.btcr);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_WRPR, py32x_info->option_bytes.wrpr);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, PY32_FLASH_CR, FLASH_OPTSTRT);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, 0x40022080, 0xFFFFFFFF);
  // if (retval != ERROR_OK)
  //   goto flash_lock;

  retval = py32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
  if (retval != ERROR_OK)
    goto flash_lock;

  retval = target_write_u32(target, PY32_FLASH_CR, FLASH_LOCK);
  if (retval != ERROR_OK)
    goto flash_lock;

flash_lock:
  {
    py32x_uninit(target);
    int retval2 = target_write_u32(target, PY32_FLASH_CR, FLASH_LOCK);
    if (retval == ERROR_OK)
      retval = retval2;
  }
  return retval;
}

static int py32x_protect_check(struct flash_bank *bank)
{
  struct target *target = bank->target;
  uint32_t protection;

  /* medium density - each bit refers to a 4 sector protection block
   * high density - each bit refers to a 2 sector protection block
   * bit 31 refers to all remaining sectors in a bank */
  int retval = target_read_u32(target, PY32_FLASH_WRPR, &protection);
  if (retval != ERROR_OK)
    return retval;

  for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
    bank->prot_blocks[i].is_protected = (protection & (1 << i)) ? 0 : 1;

  return ERROR_OK;
}

static int py32x_erase(struct flash_bank *bank, unsigned int first,
    unsigned int last)
{
  struct target *target = bank->target;

  if (bank->target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  if ((first == 0) && (last == (bank->num_sectors - 1)))
    return py32x_mass_erase(bank);

  /* unlock flash registers */
  int retval = target_write_u32(target, PY32_FLASH_KEYR, KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_KEYR, KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  retval = py32x_init(target);
  if (retval != ERROR_OK)
    goto flash_lock;

  for (unsigned int i = first; i <= last; i++) {
    retval = target_write_u32(target, PY32_FLASH_CR, FLASH_SER);
    if (retval != ERROR_OK)
      goto flash_lock;

    retval = target_write_u32(target, (uint32_t)(bank->base + bank->sectors[i].offset), 0xFFFFFFFF);
    // if (retval != ERROR_OK)
    //   goto flash_lock;
    // LOG_INFO("py32f002xx flash erase at 0x%08x", (uint32_t)(bank->base + bank->sectors[i].offset));

    retval = py32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
    if (retval != ERROR_OK)
      goto flash_lock;
  }

flash_lock:
  {
    py32x_uninit(target);
    int retval2 = target_write_u32(target, PY32_FLASH_CR, FLASH_LOCK);
    if (retval == ERROR_OK)
      retval = retval2;
  }
  return retval;
}

static int py32x_protect(struct flash_bank *bank, int set, unsigned int first,
    unsigned int last)
{
  struct target *target = bank->target;
  struct py32x_flash_bank *py32x_info = bank->driver_priv;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  int retval = py32x_erase_options(bank);
  if (retval != ERROR_OK) {
    LOG_ERROR("py32x failed to erase options");
    return retval;
  }

  for (unsigned int i = first; i <= last; i++) {
    if (set)
      py32x_info->option_bytes.wrpr &= ~(1 << i);
    else
      py32x_info->option_bytes.wrpr |= (1 << i);
  }

  return py32x_write_options(bank);
}

#if 0
static int py32x_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t pages_count)
{
	struct py32x_flash_bank *py32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct armv7m_algorithm armv7m_info;
	int retval;

	static const uint8_t py32x_flash_write_code[] = {
#include "../../../contrib/loaders/flash/py32/py32f002x.inc"
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(py32x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(py32x_flash_write_code), py32x_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	buffer_size = target_get_working_area_avail(target);
	buffer_size = MIN(pages_count * 0x80 + 8, MAX(buffer_size, 256));
	/* Normally we allocate all available working area.
	 * MIN shrinks buffer_size if the size of the written block is smaller.
	 * MAX prevents using async algo if the available working area is smaller
	 * than 256, the following allocation fails with
	 * ERROR_TARGET_RESOURCE_NOT_AVAILABLE and slow flashing takes place.
	 */

	retval = target_alloc_working_area(target, buffer_size, &source);
	/* Allocated size is always 32-bit word aligned */
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		/* target_alloc_working_area() may return ERROR_FAIL if area backup fails:
		 * convert any error to ERROR_TARGET_RESOURCE_NOT_AVAILABLE
		 */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct reg_param reg_params[6];

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (halfword-16bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */
  init_reg_param(&reg_params[5], "r5", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, py32x_info->register_base);
	buf_set_u32(reg_params[1].value, 0, 32, pages_count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);
  buf_set_u32(reg_params[5].value, 0, 32, buffer);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, pages_count, 2,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		/* Actually we just need to check for programming errors
		 * py32x_wait_status_busy also reports error and clears status bits.
		 *
		 * Target algo returns flash status in r0 only if properly finished.
		 * It is safer to re-read status register.
		 */
		int retval2 = py32x_wait_status_busy(bank, 5);
		if (retval2 != ERROR_OK)
			retval = retval2;

		LOG_ERROR("flash write failed just before address 0x%"PRIx32,
				buf_get_u32(reg_params[4].value, 0, 32));
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	return retval;
}

/** Writes a block to flash either using target algorithm
 *  or use fallback, host controlled page-by-page access.
 *  Flash controller must be unlocked before this call.
 */
static int py32x_write_block(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t address, uint32_t pages_count)
{
	struct target *target = bank->target;

	/* The flash write must be aligned to a page boundary.
	 * The flash infrastructure ensures it, do just a security check
	 */
	assert(address % 0x80 == 0);

	int retval;
	struct arm *arm = target_to_arm(target);
	if (is_arm(arm)) {
		/* try using a block write - on ARM architecture or... */
		retval = py32x_write_block_async(bank, buffer, address, pages_count);
	} else {
		/* ... RISC-V architecture */
		// retval = py32x_write_block_riscv(bank, buffer, address, pages_count);
	}

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single page accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		while (pages_count > 0) {
			retval = target_write_memory(target, address, 0x80, 1, buffer);
			if (retval != ERROR_OK)
				return retval;

			retval = py32x_wait_status_busy(bank, 5);
			if (retval != ERROR_OK)
				return retval;

			pages_count--;
			buffer += 0x80;
			address += 0x80;
		}
	}
	return retval;
}
#endif

static int py32x_write(struct flash_bank *bank, const uint8_t *buffer,
    uint32_t offset, uint32_t count)
{
  struct target *target = bank->target;

  if (bank->target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  /* The flash write must be aligned to a halfword boundary.
   * The flash infrastructure ensures it, do just a security check
   */
  assert(offset % 0x80 == 0);
  assert(count % 0x80 == 0);

  if (offset & 0x7F) {
    LOG_ERROR("offset 0x%" PRIx32 " breaks required 128-bytes alignment", offset);
    return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
  }
  
  if(count & 0x7F){
        LOG_ERROR("size 0x%" PRIx32 " breaks required 128-bytes alignment", count);
        return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
    }

  int retval, retval2;

  /* unlock flash registers */
  retval = target_write_u32(target, PY32_FLASH_KEYR, KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_KEYR, KEY2);
  if (retval != ERROR_OK)
    goto reset_pg_and_lock;

  retval = py32x_init(target);
  if (retval != ERROR_OK)
    goto reset_pg_and_lock;

  /* enable flash programming */
  // retval = target_write_u32(target, PY32_FLASH_CR, FLASH_PG);
  // if (retval != ERROR_OK)
  //   goto reset_pg_and_lock;

  /* write to flash */
  //retval = py32x_write_block(bank, buffer, bank->base + offset, count / 0x80);
  uint32_t word = 0;
  uint32_t addr = offset;
  uint32_t total = count;
  while (count) {
    // flash memory 128-bytes program        
    retval = target_write_u32(target, PY32_FLASH_CR, FLASH_PG);
    if (retval != ERROR_OK)
        goto reset_pg_and_lock;

    retval = target_write_buffer(target, (uint32_t)(bank->base + addr), 4*(0x20-1), &buffer[addr-offset+0]);
    if (retval != ERROR_OK)
      goto reset_pg_and_lock;
    addr += 4*(0x20-1);

    retval = target_write_u32(target, PY32_FLASH_CR, (FLASH_PG|FLASH_PGSTRT));
    if (retval != ERROR_OK)
      goto reset_pg_and_lock;

    word = (buffer[addr-offset+0] << 0)  | (buffer[addr-offset+1] << 8)  | (buffer[addr-offset+2] << 16) | (buffer[addr-offset+3] << 24);    
    retval = target_write_u32(target, (uint32_t)(bank->base + addr), word);
    addr += 4;

    // wait
    retval = py32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
    if (retval != ERROR_OK)
        goto reset_pg_and_lock;

    count -= 0x80;

    if (0 == ((addr - 0x80)%0x400)) {
      LOG_INFO("Programming addr 0x0%" PRIx32 " success, done %d%%.", (uint32_t)(bank->base + addr - 0x80), (total - count) * 100 / total);
    }
  }

  LOG_DEBUG("py32f002xx flash write success");

reset_pg_and_lock:
  py32x_uninit(target);
  retval2 = target_write_u32(target, PY32_FLASH_CR, FLASH_LOCK);
  if (retval == ERROR_OK)
    retval = retval2;

  return retval;
}


struct py32x_property_addr {
  uint32_t device_id;
  uint32_t flash_size;
};

static int py32x_get_property_addr(struct target *target, struct py32x_property_addr *addr)
{
  if (!target_was_examined(target)) {
    LOG_ERROR("Target not examined yet");
    return ERROR_TARGET_NOT_EXAMINED;
  }

  switch (cortex_m_get_partno_safe(target)) {
  case CORTEX_M0P_PARTNO: /* py32f002x devices */
    addr->device_id = 0x40015800;
    addr->flash_size = 0x1FFF01FC;
    return ERROR_OK;
    /* fallthrough */
  default:
    LOG_ERROR("Cannot identify target as a py32x");
    return ERROR_FAIL;
  }
}

static int py32x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
  struct target *target = bank->target;
  struct py32x_property_addr addr;

  int retval = py32x_get_property_addr(target, &addr);
  if (retval != ERROR_OK)
    return retval;

  return target_read_u32(target, addr.device_id, device_id);
}

#if 0
static int py32x_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{
  struct target *target = bank->target;
  struct py32x_property_addr addr;

  int retval = py32x_get_property_addr(target, &addr);
  if (retval != ERROR_OK)
    return retval;

  return target_read_u16(target, addr.flash_size, flash_size_in_kb);
}
#endif


static int py32x_probe(struct flash_bank *bank)
{
  struct py32x_flash_bank *py32x_info = bank->driver_priv;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t dbgmcu_idcode;
	int sector_size;
	uint32_t base_address = 0x08000000;

	py32x_info->probed = false;

	/* read py32 device id register */
	int retval = py32x_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%" PRIx32 "", dbgmcu_idcode);

    /* get flash size from target. */
	//retval = py32x_get_flash_size(bank, &flash_size_in_kb);

	/* set sector size, protection granularity and max flash size depending on family */
	// switch (dbgmcu_idcode) {
	// case 0x20220064: /* pt064 */
		sector_size = 0x1000;
		py32x_info->psector_size = 1;
		max_flash_size_in_kb = 24;

		flash_size_in_kb = 24;
	// 	break;
	// default:
	// 	LOG_WARNING("Cannot identify target as a PY32 family.");
	// 	return ERROR_FAIL;
	// }	

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("PY32 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (py32x_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = py32x_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %d KB", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of sectors */
	int num_sectors = flash_size_in_kb * 1024 / sector_size;

	/* check that calculation result makes sense */
	assert(num_sectors > 0);

	free(bank->sectors);
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	bank->base = base_address;
	bank->size = (num_sectors * sector_size);

	bank->num_sectors = num_sectors;
	bank->sectors = alloc_block_array(0, sector_size, num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* calculate number of write protection blocks */
	int num_prot_blocks = num_sectors / py32x_info->psector_size;
	if (num_prot_blocks > 32)
		num_prot_blocks = 32;

	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, py32x_info->psector_size * sector_size, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	if (num_prot_blocks == 32)
		bank->prot_blocks[31].size = (num_sectors - (31 * py32x_info->psector_size)) * sector_size;

	py32x_info->probed = true;

	return ERROR_OK;
}

static int py32x_auto_probe(struct flash_bank *bank)
{
  struct py32x_flash_bank *py32x_info = bank->driver_priv;
	if (py32x_info->probed)
		return ERROR_OK;
	return py32x_probe(bank);
}

static int get_py32x_info(struct flash_bank *bank, struct command_invocation *cmd)
{
  uint32_t dbgmcu_idcode;

  /* read py32 device id register */
  int retval = py32x_get_device_id(bank, &dbgmcu_idcode);
  if (retval != ERROR_OK)
    return retval;

  // switch (dbgmcu_idcode) {
  // case 0x20220064:
  //   break;

  // default:
  //   command_print_sameline(cmd, "Cannot identify target as a PY32F0/1/3\n");
  //   return ERROR_FAIL;
  // }
 
  command_print_sameline(cmd, "dbgmcu_idcode (0x%08x)", dbgmcu_idcode);

  return ERROR_OK;
}

COMMAND_HANDLER(py32x_handle_lock_command)
{
  struct target *target = NULL;
  struct py32x_flash_bank *py32x_info = NULL;

  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  py32x_info = bank->driver_priv;

  target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  if (py32x_erase_options(bank) != ERROR_OK) {
    command_print(CMD, "py32x failed to erase options");
    return ERROR_OK;
  }

  /* set readout protection */
  py32x_info->option_bytes.optr = ((py32x_info->option_bytes.optr & 0xFF00) | 0x55);

  if (py32x_write_options(bank) != ERROR_OK) {
    command_print(CMD, "py32x failed to lock device");
    return ERROR_OK;
  }

  command_print(CMD, "py32x locked");

  return ERROR_OK;
}

COMMAND_HANDLER(py32x_handle_unlock_command)
{
  struct target *target = NULL;

  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  if (py32x_erase_options(bank) != ERROR_OK) {
    command_print(CMD, "py32x failed to erase options");
    return ERROR_OK;
  }

  if (py32x_write_options(bank) != ERROR_OK) {
    command_print(CMD, "py32x failed to unlock device");
    return ERROR_OK;
  }

  command_print(CMD, "py32x unlocked.\n"
      "INFO: a reset or power cycle is required "
      "for the new settings to take effect.");

  return ERROR_OK;
}

COMMAND_HANDLER(py32x_handle_options_read_command)
{
  uint32_t optr, sdkr, btcr, wrpr;
  struct py32x_flash_bank *py32x_info = NULL;
  struct target *target = NULL;

  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  py32x_info = bank->driver_priv;

  target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  retval = target_read_u32(target, PY32_FLASH_OPTR, &optr);
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, PY32_FLASH_SDKR, &sdkr);
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, PY32_FLASH_BTCR, &btcr);
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, PY32_FLASH_WRPR, &wrpr);
  if (retval != ERROR_OK)
    return retval;

  command_print(CMD, "optr register = 0x%" PRIx32 "", optr);
  command_print(CMD, "sdkr register = 0x%" PRIx32 "", sdkr);
  command_print(CMD, "btcr register = 0x%" PRIx32 "", btcr);
  command_print(CMD, "wrpr register = 0x%" PRIx32 "", wrpr);

  command_print(CMD, "read protection: %s",
        ((OPTR_DEFAULT & 0xFF) == (optr & 0xFF)) ? "inactive" : "active");

  py32x_info->option_bytes.optr = optr;
  py32x_info->option_bytes.sdkr = sdkr;
  py32x_info->option_bytes.btcr = btcr;
  py32x_info->option_bytes.wrpr = wrpr;

  return ERROR_OK;
}

COMMAND_HANDLER(py32x_handle_options_write_command)
{
  struct target *target = NULL;
  struct py32x_flash_bank *py32x_info = NULL;
  uint32_t optr, sdkr, wrpr;

  if (CMD_ARGC < 2)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  py32x_info = bank->driver_priv;

  target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  retval = py32x_read_options(bank);
  if (retval != ERROR_OK)
    return retval;

  /* skip over flash bank */
  CMD_ARGC--;
  CMD_ARGV++;

  while (CMD_ARGC) {
    if (strcmp("OPTR", CMD_ARGV[0]) == 0) {
      if (CMD_ARGC < 2)
        return ERROR_COMMAND_SYNTAX_ERROR;
      COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], optr);
      CMD_ARGC--;
      CMD_ARGV++;
    } else if (strcmp("SDKR", CMD_ARGV[0]) == 0) {
      if (CMD_ARGC < 2)
        return ERROR_COMMAND_SYNTAX_ERROR;
      COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], sdkr);
      CMD_ARGC--;
      CMD_ARGV++;
    } else if (strcmp("WRPR", CMD_ARGV[0]) == 0) {
      if (CMD_ARGC < 2)
        return ERROR_COMMAND_SYNTAX_ERROR;
      COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], wrpr);
      CMD_ARGC--;
      CMD_ARGV++;
    } else
      return ERROR_COMMAND_SYNTAX_ERROR;
    CMD_ARGC--;
    CMD_ARGV++;
  }

  if (py32x_erase_options(bank) != ERROR_OK) {
    command_print(CMD, "py32x failed to erase options");
    return ERROR_OK;
  }

  py32x_info->option_bytes.optr = optr;
  py32x_info->option_bytes.sdkr = sdkr;
  py32x_info->option_bytes.wrpr = wrpr;

  if (py32x_write_options(bank) != ERROR_OK) {
    command_print(CMD, "py32x failed to write options");
    return ERROR_OK;
  }

  command_print(CMD, "py32x write options complete.\n"
        "INFO: %spower cycle is required "
        "for the new settings to take effect.",
        "'py32f002x options_load' command or ");

  return ERROR_OK;
}

COMMAND_HANDLER(py32x_handle_options_load_command)
{
  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  struct target *target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  /* unlock option flash registers */
  retval = target_write_u32(target, PY32_FLASH_KEYR, KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_KEYR, KEY2);
  if (retval != ERROR_OK) {
    (void)target_write_u32(target, PY32_FLASH_CR, FLASH_LOCK);
    return retval;
  }

  /* force re-load of option bytes - generates software reset */
  retval = target_write_u32(target, PY32_FLASH_CR, FLASH_OBL_LAUNCH);
  if (retval != ERROR_OK)
    return retval;

  return ERROR_OK;
}

static int py32x_init(struct target *target)
{
#if 0
  uint32_t hsi_trim, flash_trim;

  int retval = target_read_u32(target, PY32_RCC_ICSCR, &gui_rcc_icscr);
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, 0x1FFF0F10, &hsi_trim);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_RCC_ICSCR, ((gui_rcc_icscr&0xFFFF0000)|(hsi_trim&0x0000FFFF)));
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, 0x1FFF0F6C, &flash_trim);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_TS0, ((flash_trim>>0)&0x000000FF));
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_TS3, ((flash_trim>>8)&0x000000FF));
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_TS1, ((flash_trim>>16)&0x000001FF));
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, 0x1FFF0F70, &flash_trim);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_TS2P, ((flash_trim>>0)&0x000000FF));
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_TPS3, ((flash_trim>>16)&0x000007FF));
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, 0x1FFF0F74, &flash_trim);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_PERTPE, ((flash_trim>>0)&0x0001FFFF));
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, 0x1FFF0F78, &flash_trim);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_SMERTPE, ((flash_trim>>0)&0x0001FFFF));
  if (retval != ERROR_OK)
    return retval;

  retval = target_read_u32(target, 0x1FFF0F7C, &flash_trim);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_PRGTPE, ((flash_trim>>0)&0x0000FFFF));
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_PRETPE, ((flash_trim>>16)&0x0000FFFF));
  if (retval != ERROR_OK)
    return retval;

  return retval;
#endif
  uint32_t optr;
  int retval = target_read_u32(target, PY32_FLASH_OPTR, &optr);
  if (retval != ERROR_OK)
    return retval;
  if (0 == (optr & FLASH_OPTR_IWDG_SW))
  {
    retval = target_write_u32(target, PY32_IWDG_KR, 0x5555);
    if (retval != ERROR_OK)
      return retval;
    retval = target_write_u32(target, PY32_IWDG_PR, 0x06);
    if (retval != ERROR_OK)
      return retval;
    retval = target_write_u32(target, PY32_IWDG_RLR, 0xFFF);
    if (retval != ERROR_OK)
      return retval;
  }
  return ERROR_OK;
}

static int py32x_uninit(struct target *target)
{
  //return target_write_u32(target, PY32_RCC_ICSCR, gui_rcc_icscr);
  return ERROR_OK;
}

static int py32x_mass_erase(struct flash_bank *bank)
{
  struct target *target = bank->target;

  if (target->state != TARGET_HALTED) {
    LOG_ERROR("Target not halted");
    return ERROR_TARGET_NOT_HALTED;
  }

  /* unlock option flash registers */
  int retval = target_write_u32(target, PY32_FLASH_KEYR, KEY1);
  if (retval != ERROR_OK)
    return retval;
  retval = target_write_u32(target, PY32_FLASH_KEYR, KEY2);
  if (retval != ERROR_OK)
    goto flash_lock;

  retval = py32x_init(target);
  if (retval != ERROR_OK)
    goto flash_lock;

  /* mass erase flash memory */
  retval = target_write_u32(target, PY32_FLASH_CR, FLASH_MER);
  if (retval != ERROR_OK)
    goto flash_lock;
  retval = target_write_u32(target, 0x08000000, 0xFFFFFFFF);
  // if (retval != ERROR_OK)
  //   goto flash_lock;

  retval = py32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);

flash_lock:
  {
    py32x_uninit(target);
    int retval2 = target_write_u32(target, PY32_FLASH_CR, FLASH_LOCK);
    if (retval == ERROR_OK)
      retval = retval2;
  }
  return retval;
}

COMMAND_HANDLER(py32x_handle_mass_erase_command)
{
  if (CMD_ARGC < 1)
    return ERROR_COMMAND_SYNTAX_ERROR;

  struct flash_bank *bank;
  int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
  if (retval != ERROR_OK)
    return retval;

  retval = py32x_mass_erase(bank);
  if (retval == ERROR_OK)
    command_print(CMD, "py32x mass erase complete");
  else
    command_print(CMD, "py32x mass erase failed");

  return retval;
}

static const struct command_registration py32f002x_exec_command_handlers[] = {
  {
    .name = "lock",
    .handler = py32x_handle_lock_command,
    .mode = COMMAND_EXEC,
    .usage = "bank_id",
    .help = "Lock entire flash device.",
  },
  {
    .name = "unlock",
    .handler = py32x_handle_unlock_command,
    .mode = COMMAND_EXEC,
    .usage = "bank_id",
    .help = "Unlock entire protected flash device.",
  },
  {
    .name = "mass_erase",
    .handler = py32x_handle_mass_erase_command,
    .mode = COMMAND_EXEC,
    .usage = "bank_id",
    .help = "Erase entire flash device.",
  },
  {
    .name = "options_read",
    .handler = py32x_handle_options_read_command,
    .mode = COMMAND_EXEC,
    .usage = "bank_id",
    .help = "Read and display device option bytes.",
  },
  {
    .name = "options_write",
    .handler = py32x_handle_options_write_command,
    .mode = COMMAND_EXEC,
    .usage = "('OPTR' optr_data) "
      "('SDKR' sdkr_data) "
      "('WRPR' wrpr_data)",
    .help = "Replace bits in device option bytes.",
  },
  {
    .name = "options_load",
    .handler = py32x_handle_options_load_command,
    .mode = COMMAND_EXEC,
    .usage = "bank_id",
    .help = "Force re-load of device option bytes.",
  },
  COMMAND_REGISTRATION_DONE
};

static const struct command_registration py32f002x_command_handlers[] = {
  {
    .name = "py32f002x",
    .mode = COMMAND_ANY,
    .help = "py32f002x flash command group",
    .usage = "",
    .chain = py32f002x_exec_command_handlers,
  },
  COMMAND_REGISTRATION_DONE
};

const struct flash_driver py32f002x_flash = {
  .name = "py32f002x",
  .commands = py32f002x_command_handlers,
  .flash_bank_command = py32x_flash_bank_command,
  .erase = py32x_erase,
  .protect = py32x_protect,
  .write = py32x_write,
  .read = default_flash_read,
  .probe = py32x_probe,
  .auto_probe = py32x_auto_probe,
  .erase_check = default_flash_blank_check,
  .protect_check = py32x_protect_check,
  .info = get_py32x_info,
  .free_driver_priv = default_flash_free_driver_priv,
};
