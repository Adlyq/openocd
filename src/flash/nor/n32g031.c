// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Nationstech N32G031 Flash Driver                                      *
 *   Based on stm32f1x.c                                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

#define FLASH_REG_BASE 0x40022000

#define N32G031_FLASH_ACR     0x00
#define N32G031_FLASH_KEYR    0x04
#define N32G031_FLASH_OPTKEYR 0x08
#define N32G031_FLASH_SR      0x0C
#define N32G031_FLASH_CR      0x10
#define N32G031_FLASH_AR      0x14
#define N32G031_FLASH_OBR     0x1C
#define N32G031_FLASH_WRPR    0x20
#define N32G031_FLASH_FLAG    0x34

/* FLASH_CR register bits */
#define FLASH_PG          (1 << 0)
#define FLASH_PER         (1 << 1)
#define FLASH_MER         (1 << 2)
#define FLASH_OPTPG       (1 << 4)
#define FLASH_OPTER       (1 << 5)
#define FLASH_STRT        (1 << 6)
#define FLASH_LOCK        (1 << 7)

/* FLASH_SR register bits */
#define FLASH_BSY         (1 << 0)
#define FLASH_PGERR       (1 << 2)
#define FLASH_WRPRTERR    (1 << 4)
#define FLASH_EOP         (1 << 5)

/* register unlock keys */
#define KEY1              0x45670123
#define KEY2              0xCDEF89AB

#define N32G031_FLASH_PAGE_SIZE 512

struct n32g031_flash_bank {
	bool probed;
	uint32_t register_base;
	uint32_t idcode;
};

struct n32g031_trim_state {
	bool applied;
	uint32_t trimr0;
	uint32_t trimr1;
};

static inline int n32g031_get_flash_reg(uint32_t reg)
{
	return reg + FLASH_REG_BASE;
}

static inline int n32g031_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, n32g031_get_flash_reg(N32G031_FLASH_SR), status);
}

static int n32g031_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = n32g031_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		if ((status & FLASH_BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FLASH_BUSY;
		}
		alive_sleep(1);
	}

	if (status & FLASH_WRPRTERR) {
		LOG_ERROR("n32g031 device protected");
		retval = ERROR_FLASH_PROTECTED;
	}

	if (status & FLASH_PGERR) {
		LOG_ERROR("n32g031 device programming failed / flash not erased");
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

	/* Clear errors (using N32G031 standard clear mask 0xB4) */
	if (status & (FLASH_WRPRTERR | FLASH_PGERR | FLASH_EOP)) {
		target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_SR), 0xB4);
	}

	return retval;
}

static int n32g031_apply_erase_trim(struct target *target, struct n32g031_trim_state *state)
{
	state->applied = false;
	uint32_t flash_flag;
	int retval = target_read_u32(target, n32g031_get_flash_reg(N32G031_FLASH_FLAG), &flash_flag);
	if (retval != ERROR_OK)
		return retval;

	if (flash_flag == 0xF7FFF7FF) {
		uint32_t trimr0, trimr1;
		retval = target_read_u32(target, 0x40001800, &trimr0);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u32(target, 0x40001804, &trimr1);
		if (retval != ERROR_OK)
			return retval;

		state->trimr0 = trimr0;
		state->trimr1 = trimr1;
		state->applied = true;

		uint32_t flash_vol = trimr0 & 0x00000700;
		if (flash_vol < 0x00000300)
			flash_vol = 0;
		else
			flash_vol -= 0x00000200;

		uint32_t val_r0 = (trimr0 & ~0x00000700) | flash_vol;
		retval = target_write_u32(target, 0x40001800, val_r0);
		if (retval != ERROR_OK)
			return retval;

		uint32_t val_r1 = trimr1 | 0x0000FFC0;
		retval = target_write_u32(target, 0x40001804, val_r1);
		if (retval != ERROR_OK) {
			target_write_u32(target, 0x40001800, trimr0);
			return retval;
		}

		alive_sleep(1);
	}
	return ERROR_OK;
}

static int n32g031_apply_program_trim(struct target *target, struct n32g031_trim_state *state)
{
	state->applied = false;
	uint32_t flash_flag;
	int retval = target_read_u32(target, n32g031_get_flash_reg(N32G031_FLASH_FLAG), &flash_flag);
	if (retval != ERROR_OK)
		return retval;

	if (flash_flag == 0xF7FFF7FF) {
		uint32_t trimr0, trimr1;
		retval = target_read_u32(target, 0x40001800, &trimr0);
		if (retval != ERROR_OK)
			return retval;
		retval = target_read_u32(target, 0x40001804, &trimr1);
		if (retval != ERROR_OK)
			return retval;

		state->trimr0 = trimr0;
		state->trimr1 = trimr1;
		state->applied = true;

		uint32_t flash_vol = trimr0 & 0x00000700;
		if (flash_vol > 0x00000400)
			flash_vol = 0x00000700;
		else
			flash_vol += 0x00000200;

		uint32_t val_r0 = (trimr0 & ~0x00000700) | flash_vol;
		retval = target_write_u32(target, 0x40001800, val_r0);
		if (retval != ERROR_OK)
			return retval;

		uint32_t val_r1 = trimr1 & ~0x0000FFC0;
		retval = target_write_u32(target, 0x40001804, val_r1);
		if (retval != ERROR_OK) {
			target_write_u32(target, 0x40001800, trimr0);
			return retval;
		}

		alive_sleep(1);
	}
	return ERROR_OK;
}

static int n32g031_restore_trim(struct target *target, struct n32g031_trim_state *state)
{
	if (!state->applied)
		return ERROR_OK;

	int retval = target_write_u32(target, 0x40001800, state->trimr0);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, 0x40001804, state->trimr1);
	return retval;
}

static int n32g031_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Unlock Flash */
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* Clear errors and status flags */
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_SR), 0xB4);
	if (retval != ERROR_OK)
		return retval;

	struct n32g031_trim_state trim_state = {0};

	/* Apply trimming if we are erasing the first sector */
	if (first == 0) {
		retval = n32g031_apply_erase_trim(target, &trim_state);
		if (retval != ERROR_OK)
			goto erase_error;
	}

	for (unsigned int i = first; i <= last; i++) {
		uint32_t address = bank->base + i * N32G031_FLASH_PAGE_SIZE;

		retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_CR), FLASH_PER);
		if (retval != ERROR_OK)
			goto erase_error;
		retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_AR), address);
		if (retval != ERROR_OK)
			goto erase_error;
		retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_CR), FLASH_PER | FLASH_STRT);
		if (retval != ERROR_OK)
			goto erase_error;

		retval = n32g031_wait_status_busy(bank, 100);
		if (retval != ERROR_OK)
			goto erase_error;
	}

erase_error:
	n32g031_restore_trim(target, &trim_state);

	/* Lock Flash */
	target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_CR), FLASH_LOCK);

	return retval;
}

static int n32g031_write_block_async(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t words_count)
{
	struct target *target = bank->target;
	uint32_t buffer_size;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct armv7m_algorithm armv7m_info;
	int retval;

	/* Custom 32-bit word block write loader */
	static const uint8_t n32g031_flash_write_code[] = {
		0x16, 0x68, 0x00, 0x2e, 0x18, 0xd0, 0x55, 0x68, 0xb5, 0x42, 0xf9, 0xd0,
		0x2e, 0x68, 0x26, 0x60, 0x04, 0x35, 0x04, 0x34, 0xc6, 0x68, 0x01, 0x27,
		0x3e, 0x42, 0xfb, 0xd1, 0x14, 0x27, 0x3e, 0x42, 0x08, 0xd1, 0x9d, 0x42,
		0x01, 0xd3, 0x15, 0x46, 0x08, 0x35, 0x55, 0x60, 0x49, 0x1e, 0x00, 0x29,
		0x02, 0xd0, 0xe5, 0xe7, 0x00, 0x20, 0x50, 0x60, 0x30, 0x46, 0x00, 0xbe
	};

	if (target_alloc_working_area(target, sizeof(n32g031_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(n32g031_flash_write_code), n32g031_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	buffer_size = target_get_working_area_avail(target);
	buffer_size = MIN(words_count * 4 + 8, MAX(buffer_size, 256));

	retval = target_alloc_working_area(target, buffer_size, &source);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct reg_param reg_params[5];

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (word-32bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, FLASH_REG_BASE);
	buf_set_u32(reg_params[1].value, 0, 32, words_count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, words_count, 4,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		int retval2 = n32g031_wait_status_busy(bank, 5);
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

static int n32g031_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3 || count & 0x3) {
		LOG_ERROR("write offset and count must be aligned to a word boundary");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* Unlock Flash */
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	struct n32g031_trim_state trim_state = {0};

	/* Apply trimming if we are programming the first sector */
	if (offset < N32G031_FLASH_PAGE_SIZE) {
		retval = n32g031_apply_program_trim(target, &trim_state);
		if (retval != ERROR_OK)
			goto write_error;
	}

	/* Clear errors */
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_SR), 0xB4);
	if (retval != ERROR_OK)
		goto write_error;

	/* Enable programming */
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_CR), FLASH_PG);
	if (retval != ERROR_OK)
		goto write_error;

	/* Run block write */
	retval = n32g031_write_block_async(bank, buffer, bank->base + offset, count / 4);

write_error:
	n32g031_restore_trim(target, &trim_state);

	/* Lock Flash */
	target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_CR), FLASH_LOCK);

	return retval;
}

static int n32g031_probe(struct flash_bank *bank)
{
	struct n32g031_flash_bank *n32g031_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t idcode;

	n32g031_info->probed = false;

	/* Read DBGMCU_ID from 0x1FFFF508 */
	int retval = target_read_u32(target, 0x1FFFF508, &idcode);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read DBGMCU_ID register");
		return retval;
	}

	n32g031_info->idcode = idcode;

	/* Check chip model: (idcode >> 8) & 0xFF should be 0x03 */
	uint32_t model = (idcode >> 8) & 0xFF;
	if (model != 0x03) {
		LOG_WARNING("Unexpected N32 DBGMCU_ID: 0x%08" PRIx32 " (Model: 0x%02" PRIx32 ")", idcode, model);
	}

	/* Flash size in KB = ((idcode >> 16) & 0xF) * 16 */
	uint32_t flash_size_in_kb = ((idcode >> 16) & 0xF) * 16;
	if (flash_size_in_kb == 0) {
		/* Fallback to 64KB if probe capacity is zero/invalid */
		flash_size_in_kb = 64;
	}

	LOG_INFO("N32G031 detected. Flash size: %u KB, SRAM: %u KB", flash_size_in_kb, ((idcode >> 20) & 0xF) * 8);

	bank->size = flash_size_in_kb * 1024;
	bank->num_sectors = bank->size / N32G031_FLASH_PAGE_SIZE;
	bank->sectors = alloc_block_array(0, N32G031_FLASH_PAGE_SIZE, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	n32g031_info->probed = true;

	return ERROR_OK;
}

static int n32g031_auto_probe(struct flash_bank *bank)
{
	struct n32g031_flash_bank *n32g031_info = bank->driver_priv;
	if (n32g031_info->probed)
		return ERROR_OK;
	return n32g031_probe(bank);
}

static int n32g031_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Unlock Flash */
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* Clear status and errors */
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_SR), 0xB4);
	if (retval != ERROR_OK)
		return retval;

	/* Mass Erase */
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_CR), FLASH_MER);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_CR), FLASH_MER | FLASH_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = n32g031_wait_status_busy(bank, 500);
	if (retval != ERROR_OK)
		return retval;

	/* Lock Flash */
	target_write_u32(target, n32g031_get_flash_reg(N32G031_FLASH_CR), FLASH_LOCK);

	return ERROR_OK;
}

COMMAND_HANDLER(n32g031_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = n32g031_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "n32g031 mass erase complete");
	else
		command_print(CMD, "n32g031 mass erase failed");

	return retval;
}

static const struct command_registration n32g031_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = n32g031_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration n32g031_command_handlers[] = {
	{
		.name = "n32g031",
		.mode = COMMAND_ANY,
		.help = "n32g031 flash command group",
		.usage = "",
		.chain = n32g031_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

FLASH_BANK_COMMAND_HANDLER(n32g031_flash_bank_command)
{
	struct n32g031_flash_bank *n32g031_info;

	n32g031_info = malloc(sizeof(struct n32g031_flash_bank));
	if (!n32g031_info)
		return ERROR_FAIL;

	bank->driver_priv = n32g031_info;
	n32g031_info->probed = false;
	n32g031_info->register_base = FLASH_REG_BASE;

	bank->write_start_alignment = bank->write_end_alignment = 4;

	return ERROR_OK;
}

const struct flash_driver n32g031_flash = {
	.name = "n32g031",
	.commands = n32g031_command_handlers,
	.flash_bank_command = n32g031_flash_bank_command,
	.erase = n32g031_erase,
	.write = n32g031_write,
	.read = default_flash_read,
	.probe = n32g031_probe,
	.auto_probe = n32g031_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
