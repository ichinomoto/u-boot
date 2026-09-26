/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __POMERA_BT_ADDRESS_H
#define __POMERA_BT_ADDRESS_H

/* Optional fixup of the Linux DTB; failure must not prevent SD boot. */
void pomera_setup_bt_address(void *fdt, size_t capacity);

#endif
