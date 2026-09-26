// SPDX-License-Identifier: GPL-2.0+
/* Pass the factory Bluetooth address to Linux without exposing eMMC partitions. */
#include <blk.h>
#include <dm.h>
#include <ext4fs.h>
#include <hexdump.h>
#include <malloc.h>
#include <memalign.h>
#include <mmc.h>
#include <part.h>
#include <asm/unaligned.h>
#include <linux/ctype.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>

#include "bt_address.h"

#define POMERA_SECTOR_SIZE	512
#define POMERA_PARM_BASE		0x2000
#define POMERA_PARM_STRIDE	0x400
#define POMERA_PARM_COPIES	8
#define POMERA_PARM_SIZE		SZ_64K
#define POMERA_BT_NODE		"/serial@20060000/bluetooth"

/* Stock PARM uses the same nonstandard polynomial as the Rockchip loader. */
static u32 pomera_parm_crc(const u8 *data, u32 size)
{
	u32 crc = 0;
	int i;

	while (size--) {
		crc ^= (u32)*data++ << 24;
		for (i = 0; i < 8; i++)
			crc = (crc << 1) ^ ((crc & BIT(31)) ? 0x04c10db7 : 0);
	}
	return crc;
}

static int pomera_hex(const char **text, u32 *value)
{
	const char *p = *text;
	u32 result = 0;
	int digit, count = 0;

	if (p[0] != '0' || (p[1] != 'x' && p[1] != 'X'))
		return -EINVAL;
	p += 2;
	while ((digit = hex_to_bin(*p)) >= 0) {
		if (++count > 8)
			return -ERANGE;
		result = (result << 4) | digit;
		p++;
	}
	if (!count)
		return -EINVAL;
	*text = p;
	*value = result;
	return 0;
}

static int pomera_find_sys_info(const char *text, lbaint_t sectors,
				struct disk_partition *part)
{
	const char *p, *end, *name;
	bool found = false, unlimited;
	u32 size, offset;
	u64 start;
	int ret;

	p = strstr(text, "CMDLINE:");
	if (!p || (p != text && p[-1] != '\n'))
		return -EINVAL;
	end = strchr(p, '\n');
	p = strstr(p, "mtdparts=rk29xxnand:");
	if (!p || (end && p >= end))
		return -EINVAL;
	p += strlen("mtdparts=rk29xxnand:");
	do {
		unlimited = *p == '-';
		size = 0;
		if (unlimited) {
			p++;
		} else {
			ret = pomera_hex(&p, &size);
			if (ret || !size)
				return -EINVAL;
		}
		if (*p++ != '@' || pomera_hex(&p, &offset) || *p++ != '(')
			return -EINVAL;
		name = p;
		while (*p && *p != ')' && !isspace(*p))
			p++;
		if (*p != ')')
			return -EINVAL;
		if (p - name == 8 && !memcmp(name, "sys_info", 8)) {
			/* Vendor offsets are sectors relative to raw eMMC LBA 0x2000. */
			start = (u64)offset + POMERA_PARM_BASE;
			if (found || unlimited || start < 2 * POMERA_PARM_BASE ||
			    start >= sectors || size > sectors - start)
				return -EINVAL;
			part->start = start;
			part->size = size;
			part->blksz = POMERA_SECTOR_SIZE;
			found = true;
		}
		p++;
		if (!*p || isspace(*p))
			break;
		if (*p++ != ',')
			return -EINVAL;
		if (!*p || isspace(*p))
			return -EINVAL;
	} while (*p);

	return found ? 0 : -ENOENT;
}

static int pomera_read_sys_info(struct blk_desc *desc, struct disk_partition *part)
{
	lbaint_t sector;
	u32 size, blocks;
	u8 *buf;
	int i, ret = -EINVAL;

	buf = memalign(ARCH_DMA_MINALIGN, POMERA_PARM_SIZE);
	if (!buf)
		return -ENOMEM;
	for (i = 0; i < POMERA_PARM_COPIES; i++) {
		sector = POMERA_PARM_BASE + i * POMERA_PARM_STRIDE;
		if (sector >= desc->lba || blk_dread(desc, sector, 1, buf) != 1 ||
		    memcmp(buf, "PARM", 4))
			continue;
		size = get_unaligned_le32(buf + 4);
		if (!size || size > POMERA_PARM_SIZE - 12)
			continue;
		blocks = DIV_ROUND_UP(size + 12, POMERA_SECTOR_SIZE);
		if (blocks > desc->lba - sector ||
		    blk_dread(desc, sector, blocks, buf) != blocks)
			continue;
		if (pomera_parm_crc(buf + 8, size) !=
		    get_unaligned_le32(buf + 8 + size))
			continue;
		/* Permit a trailing NUL, but not a hidden second command line. */
		if (memchr(buf + 8, '\0', size - 1))
			continue;
		buf[8 + size] = '\0';
		ret = pomera_find_sys_info((char *)buf + 8, desc->lba, part);
		break;
	}
	free(buf);
	return ret;
}

/* Addresses here, including DT properties, are least-significant byte first. */
static bool pomera_valid_bdaddr(const u8 *addr)
{
	static const u8 invalid[][6] = {
		{ 0, 0, 0, 0, 0, 0 },
		{ 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa },
		{ 0x55, 0x44, 0x33, 0x22, 0x11, 0x00 },
		{ 0x00, 0xa0, 0x02, 0x70, 0x20, 0x00 },
		{ 0x00, 0x00, 0xa0, 0x02, 0x70, 0x20 },
		{ 0xac, 0x1f, 0x12, 0xa0, 0x43, 0x43 },
	};
	int i;

	if (addr[5] & 1)
		return false;
	for (i = 0; i < ARRAY_SIZE(invalid); i++) {
		if (!memcmp(addr, invalid[i], 6))
			return false;
	}
	return true;
}

static int pomera_parse_bdaddr(const char *text, size_t size, u8 *addr)
{
	int i;

	if (size && text[size - 1] == '\n')
		size--;
	if (size && text[size - 1] == '\r')
		size--;
	if (size != 17)
		return -EINVAL;
	for (i = 0; i < 6; i++) {
		if ((i < 5 && text[i * 3 + 2] != ':') ||
		    hex2bin(&addr[5 - i], text + i * 3, 1))
			return -EINVAL;
	}
	return pomera_valid_bdaddr(addr) ? 0 : -EINVAL;
}

static int pomera_read_bdaddr(struct blk_desc *desc, u8 *addr)
{
	struct disk_partition part = {};
	loff_t size, actual;
	char text[19];
	int ret;

	ret = pomera_read_sys_info(desc, &part);
	if (ret)
		return ret;
	/* The ext reader bounds every access to this partition; no writes. */
	ret = ext4fs_probe(desc, &part);
	if (ret)
		return ret;
	ret = ext4fs_open("/bt_mac.dat", &size);
	if (ret)
		goto out;
	if (size < 17 || size > sizeof(text)) {
		ret = -EINVAL;
		goto out;
	}
	ret = ext4fs_read(text, 0, size, &actual);
	if (!ret)
		ret = actual == size ? pomera_parse_bdaddr(text, size, addr) : -EIO;
out:
	ext4fs_close();
	return ret;
}

void pomera_setup_bt_address(void *fdt, size_t capacity)
{
	struct udevice *dev;
	struct blk_desc *desc;
	struct mmc *mmc;
	const u8 *existing;
	u8 addr[6];
	int node, len, ret;

	node = fdt_path_offset(fdt, POMERA_BT_NODE);
	if (node < 0)
		return;
	existing = fdt_getprop(fdt, node, "local-bd-address", &len);
	if (existing && len == sizeof(addr) && pomera_valid_bdaddr(existing))
		return;
	ret = uclass_get_device_by_seq(UCLASS_MMC, 0, &dev);
	if (ret)
		goto fail;
	mmc = mmc_get_mmc_dev(dev);
	ret = mmc_init(mmc);
	if (ret)
		goto fail;
	desc = mmc_get_blk_desc(mmc);
	/* Never read from the SD card or an eMMC boot/RPMB hardware partition. */
	if (IS_SD(mmc) || !desc || desc->hwpart ||
	    desc->blksz != POMERA_SECTOR_SIZE) {
		ret = -ENODEV;
		goto fail;
	}
	ret = pomera_read_bdaddr(desc, addr);
	if (ret)
		goto fail;
	if (capacity < fdt_totalsize(fdt) ||
	    capacity - fdt_totalsize(fdt) < 128) {
		ret = -ENOSPC;
		goto fail;
	}
	ret = fdt_open_into(fdt, fdt, fdt_totalsize(fdt) + 128);
	if (ret)
		goto fail;
	node = fdt_path_offset(fdt, POMERA_BT_NODE);
	ret = fdt_setprop(fdt, node, "local-bd-address", addr, sizeof(addr));
	if (ret)
		goto fail;
	puts("Pomera: Bluetooth address loaded from eMMC sys_info\n");
	return;
fail:
	printf("Pomera: factory Bluetooth address unavailable (%d), continuing boot\n",
	       ret);
}
