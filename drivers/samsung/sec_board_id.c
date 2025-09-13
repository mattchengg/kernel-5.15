// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Samsung board id helper.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/bootconfig.h>


static int board_id = 0;

// A546S support Low-Dropout (LDO) regulator.
int sec_board_support_ldo(void)
{
	if (board_id == 1)
		return 1;

	return 0;
}
EXPORT_SYMBOL(sec_board_support_ldo);

// A546S support Quantum HW random number generator.
int sec_board_support_qrng(void)
{
	if (board_id == 1)
		return 1;

	return 0;
}
EXPORT_SYMBOL(sec_board_support_qrng);

void __init sec_board_id_setup(void)
{
	char *value;

	value = (char *)xbc_find_value("androidboot.bootloader", NULL);

	if (value)
		if(strstr(value, "A546S"))
			board_id = 1;

		pr_debug("%s: bootloader: %s\n", __func__,  value);
}

int __init sec_board_id_init(void)
{
	sec_board_id_setup();
	return 0;
}

module_init(sec_board_id_init);
