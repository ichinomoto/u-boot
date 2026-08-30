// SPDX-License-Identifier: GPL-2.0+
/* DM200 direct boot and legacy eMMC U-Boot chainloader. */
#include <blk.h>
#include <command.h>
#include <cpu_func.h>
#include <dm.h>
#include <mapmem.h>
#include <mmc.h>
#include <memalign.h>
#include <asm/unaligned.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>

#define SECTOR_SIZE	512
#define KERNEL_ADDR	0x62000000
#define FDT_ADDR	0x61f00000
#define LEGACY_UBOOT_ADDR	0x60200000
#define LEGACY_UBOOT_SECTOR	0x4000
#define LOADER_HEADER_SIZE	2048
#define LOADER_HEADER_SECTORS	(LOADER_HEADER_SIZE / SECTOR_SIZE)
#define LOADER_SLOT_SIZE	SZ_1M

struct dm200_layout {
	lbaint_t kernel;
	u32 kernel_sectors;
	lbaint_t resource;
	u32 resource_sectors;
};

/* Absolute sectors: original DM200 layout plus the 4 MiB SD firmware base. */
static const struct dm200_layout sd_layout = {
	0x6000, 0x6000, 0xc000, 0x3000,
};

static const struct dm200_layout recovery_layout = {
	0x3f8000, 0x6000, 0xa000, 0x3000,
};

static int read_sectors(struct blk_desc *desc, lbaint_t start,
			u32 count, void *buf)
{
	if (start >= desc->lba || count > desc->lba - start)
		return -EINVAL;
	return blk_dread(desc, start, count, buf) == count ? 0 : -EIO;
}

static u32 rockchip_crc(const u8 *data, u32 size)
{
	u32 crc = 0, i;

	while (size--) {
		crc ^= (u32)*data++ << 24;
		for (i = 0; i < 8; i++)
			crc = (crc << 1) ^ ((crc & BIT(31)) ? 0x04c10db7 : 0);
	}
	return crc;
}

static int load_legacy_uboot(struct blk_desc *desc)
{
	ALLOC_CACHE_ALIGN_BUFFER(u8, header, LOADER_HEADER_SIZE);
	u32 load_addr, size, crc, blocks;
	void *buf;
	int ret;

	ret = read_sectors(desc, LEGACY_UBOOT_SECTOR,
			   LOADER_HEADER_SECTORS, header);
	if (ret)
		return ret;
	if (memcmp(header, "LOADER  ", 8))
		return -EINVAL;

	load_addr = get_unaligned_le32(header + 16);
	size = get_unaligned_le32(header + 20);
	crc = get_unaligned_le32(header + 24);
	if (load_addr != LEGACY_UBOOT_ADDR || !size ||
	    size > LOADER_SLOT_SIZE - LOADER_HEADER_SIZE)
		return -EINVAL;

	blocks = DIV_ROUND_UP(size, SECTOR_SIZE);
	buf = map_sysmem(load_addr, blocks * SECTOR_SIZE);
	ret = read_sectors(desc, LEGACY_UBOOT_SECTOR + LOADER_HEADER_SECTORS,
			   blocks, buf);
	if (ret)
		goto out;
	if (rockchip_crc(buf, size) != crc) {
		ret = -EBADMSG;
		goto out;
	}

	flush_cache(load_addr, blocks * SECTOR_SIZE);
	invalidate_icache_all();
	printf("DM200: legacy eMMC U-Boot loaded at 0x%08x (%u bytes)\n",
	       load_addr, size);

out:
	unmap_sysmem(buf);
	return ret;
}

static int load_krnl(struct blk_desc *desc, lbaint_t sector,
		     u32 sectors, ulong addr, u32 *size)
{
	ALLOC_CACHE_ALIGN_BUFFER(u8, header, SECTOR_SIZE);
	u8 *buf = map_sysmem(addr, SZ_32M);
	u32 len, blocks;
	int ret;

	ret = read_sectors(desc, sector, 1, header);
	if (ret)
		goto out;
	if (memcmp(header, "KRNL", 4)) {
		ret = -EINVAL;
		goto out;
	}
	len = get_unaligned_le32(header + 4);
	if (!len || len > SZ_32M - 12) {
		ret = -EFBIG;
		goto out;
	}
	blocks = DIV_ROUND_UP(len + 12, SECTOR_SIZE);
	if (blocks > sectors) {
		ret = -EFBIG;
		goto out;
	}
	ret = read_sectors(desc, sector, blocks, buf);
	if (ret)
		goto out;
	if (rockchip_crc(buf + 8, len) != get_unaligned_le32(buf + 8 + len)) {
		ret = -EBADMSG;
		goto out;
	}
	memmove(buf, buf + 8, len);
	*size = len;
	ret = 0;

out:
	unmap_sysmem(buf);
	return ret;
}

static int load_resource_fdt(struct blk_desc *desc, lbaint_t sector,
			     u32 sectors)
{
	ALLOC_CACHE_ALIGN_BUFFER(u8, header, SECTOR_SIZE);
	ALLOC_CACHE_ALIGN_BUFFER(u8, entry, SECTOR_SIZE);
	u32 count, table, i, offset, size, blocks;
	void *fdt = map_sysmem(FDT_ADDR, SZ_1M);
	int ret;

	ret = read_sectors(desc, sector, 1, header);
	if (ret)
		return ret;
	if (memcmp(header, "RSCE", 4) || get_unaligned_le16(header + 4) ||
	    get_unaligned_le16(header + 6) || header[8] != 1 || header[10] != 1)
		return -EINVAL;
	table = header[9];
	count = get_unaligned_le32(header + 12);
	if (!table || table >= sectors || count > sectors - table)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		ret = read_sectors(desc, sector + table + i, 1, entry);
		if (ret)
			return ret;
		if (memcmp(entry, "ENTR", 4))
			return -EINVAL;
		if (strncmp((char *)entry + 4, "rk-kernel.dtb", 256))
			continue;
		offset = get_unaligned_le32(entry + 260);
		size = get_unaligned_le32(entry + 264);
		if (size < sizeof(struct fdt_header) || size > SZ_1M)
			return -EFBIG;
		blocks = DIV_ROUND_UP(size, SECTOR_SIZE);
		if (offset < table + count || offset >= sectors ||
		    blocks > sectors - offset)
			return -EINVAL;
		ret = read_sectors(desc, sector + offset, blocks, fdt);
		if (ret)
			return ret;
		ret = fdt_check_full(fdt, size);
		unmap_sysmem(fdt);
		return ret;
	}
	unmap_sysmem(fdt);
	return -ENOENT;
}

static int do_dm200boot(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	const struct dm200_layout *layout = &sd_layout;
	const char *source = argc == 2 ? argv[1] : "sd";
	struct udevice *dev;
	struct blk_desc *desc;
	struct mmc *mmc;
	u32 kernel_size;
	int seq = 1, ret;
	bool chainload_legacy = false;
	unsigned long legacy_rc;
	char command[96];

	if (!strcmp(source, "emmc")) {
		seq = 0;
		chainload_legacy = true;
	} else if (!strcmp(source, "recovery")) {
		seq = 0;
		layout = &recovery_layout;
	} else if (strcmp(source, "sd")) {
		return CMD_RET_USAGE;
	}
	if (chainload_legacy)
		printf("DM200: boot from %s (mmc %d), legacy U-Boot\n",
		       source, seq);
	else
		printf("DM200: boot from %s (mmc %d), direct rootfs\n",
		       source, seq);
	ret = uclass_get_device_by_seq(UCLASS_MMC, seq, &dev);
	if (ret)
		goto fail;
	mmc = mmc_get_mmc_dev(dev);
	ret = mmc_init(mmc);
	if (ret)
		goto fail;
	desc = mmc_get_blk_desc(mmc);
	if (!desc || desc->blksz != SECTOR_SIZE) {
		ret = -EINVAL;
		goto fail;
	}
	if (chainload_legacy) {
		puts("DM200: loading legacy eMMC U-Boot\n");
		ret = load_legacy_uboot(desc);
		if (ret)
			goto fail;

		/* The legacy image is ARM code; do not use the go command here. */
		legacy_rc = ((unsigned long (*)(int, char *const []))
				(ulong)LEGACY_UBOOT_ADDR)(0, NULL);
		printf("DM200: legacy eMMC U-Boot returned (0x%lx)\n",
		       legacy_rc);
		return CMD_RET_FAILURE;
	}
	ret = load_krnl(desc, layout->kernel, layout->kernel_sectors,
			KERNEL_ADDR, &kernel_size);
	if (ret)
		goto fail;
	ret = load_resource_fdt(desc, layout->resource, layout->resource_sectors);
	if (ret)
		goto fail;
	printf("DM200: kernel %u bytes, separate resource DTB, no initramfs\n",
	       kernel_size);

	/* '-' tells bootz that no external initramfs is present. */
	snprintf(command, sizeof(command), "bootz %x - %x",
		 KERNEL_ADDR, FDT_ADDR);
	return run_command(command, 0) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;

fail:
	printf("DM200: image load failed (%d)\n", ret);
	return CMD_RET_FAILURE;
}

U_BOOT_CMD(dm200boot, 2, 0, do_dm200boot,
	   "boot DM200 directly from SD or chainload legacy eMMC U-Boot",
	   "[sd|emmc|recovery]\n"
	   "    sd: SD direct rootfs (default); emmc: chainload legacy U-Boot;\n"
	   "    recovery: direct boot from the original recovery layout");
