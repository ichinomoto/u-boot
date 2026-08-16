/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __POMERA_DM200_H
#define __POMERA_DM200_H

#define BOOT_TARGETS "mmc1"
#include <configs/rk3128_common.h>

#undef CFG_EXTRA_ENV_SETTINGS
#define CFG_EXTRA_ENV_SETTINGS \
	ENV_MEM_LAYOUT_SETTINGS \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"partitions=" PARTS_DEFAULT \
	"boot_targets=" BOOT_TARGETS "\0" \
	"stdout=serial,vidconsole\0" \
	"stderr=serial,vidconsole\0"

#endif
