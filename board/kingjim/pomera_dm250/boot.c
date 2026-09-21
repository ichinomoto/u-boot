// SPDX-License-Identifier: GPL-2.0+
/* DM250 direct boot and legacy eMMC U-Boot chainloader. */
#include <blk.h>
#include <backlight.h>
#include <command.h>
#include <cpu_func.h>
#include <dm.h>
#include <generic-phy.h>
#include <mapmem.h>
#include <mmc.h>
#include <memalign.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>

#define SECTOR_SIZE		512
#define KERNEL_ADDR		0x62000000
#define FDT_ADDR		0x61f00000
#define LEGACY_UBOOT_ADDR	0x60200000
#define LEGACY_UBOOT_SECTOR	0x4000
#define LOADER_HEADER_SIZE	2048
#define LOADER_HEADER_SECTORS	(LOADER_HEADER_SIZE / SECTOR_SIZE)
#define LOADER_SLOT_SIZE	SZ_1M

/* RK3128 display registers used to leave the panel in a known-off state. */
#define DM250_VOP_BASE			0x1010e000
#define DM250_VOP_SYS_CTRL		0x00
#define DM250_VOP_AXI_BUS_CTRL		0x2c
#define DM250_VOP_REG_CFG_DONE		0x90
#define DM250_VOP_WIN0_EN		BIT(0)
#define DM250_VOP_STANDBY		BIT(30)
#define DM250_VOP_OUTPUT_CLK_EN		(BIT(26) | BIT(24) | BIT(22))
#define DM250_GRF_LVDS_CON0		0x20008150
#define DM250_LVDS_CON0_WMSK		0x03cf

struct dm250_layout {
	lbaint_t kernel;
	u32 kernel_sectors;
	lbaint_t resource;
	u32 resource_sectors;
};

/* Initial DM250 SD images use the same absolute layout as DM200. */
static const struct dm250_layout sd_layout = {
	0x6000, 0x6000, 0xc000, 0x3000,
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
	printf("DM250: legacy eMMC U-Boot loaded at 0x%08x (%u bytes)\n",
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
		goto out;
	if (memcmp(header, "RSCE", 4) || get_unaligned_le16(header + 4) ||
	    get_unaligned_le16(header + 6) || header[8] != 1 || header[10] != 1) {
		ret = -EINVAL;
		goto out;
	}
	table = header[9];
	count = get_unaligned_le32(header + 12);
	if (!table || table >= sectors || count > sectors - table) {
		ret = -EINVAL;
		goto out;
	}
	for (i = 0; i < count; i++) {
		ret = read_sectors(desc, sector + table + i, 1, entry);
		if (ret)
			goto out;
		if (memcmp(entry, "ENTR", 4)) {
			ret = -EINVAL;
			goto out;
		}
		if (strncmp((char *)entry + 4, "rk-kernel.dtb", 256))
			continue;
		offset = get_unaligned_le32(entry + 260);
		size = get_unaligned_le32(entry + 264);
		if (size < sizeof(struct fdt_header) || size > SZ_1M) {
			ret = -EFBIG;
			goto out;
		}
		blocks = DIV_ROUND_UP(size, SECTOR_SIZE);
		if (offset < table + count || offset >= sectors ||
		    blocks > sectors - offset) {
			ret = -EINVAL;
			goto out;
		}
		ret = read_sectors(desc, sector + offset, blocks, fdt);
		if (ret)
			goto out;
		ret = fdt_check_full(fdt, size);
		goto out;
	}
	ret = -ENOENT;

out:
	unmap_sysmem(fdt);
	return ret;
}

/*
 * Leave the display in a known-off state before handing control to the
 * legacy U-Boot.  The next boot stage reinitialises the VOP/LVDS pipeline.
 * This deliberately favours a short black interval over a seamless
 * U-Boot-to-U-Boot transition.
 */
static void dm250_display_shutdown(void)
{
	struct udevice *backlight, *display;
	struct phy dphy;
	u32 val;
	int ret;
	bool have_dphy = false;

	ret = uclass_get_device(UCLASS_PANEL_BACKLIGHT, 0, &backlight);
	if (!ret) {
		ret = backlight_set_brightness(backlight, BACKLIGHT_OFF);
		if (ret)
			printf("DM250: failed to turn off backlight (%d)\n", ret);
	} else {
		printf("DM250: backlight unavailable during shutdown (%d)\n", ret);
	}

	/* Stop the active window and request VOP standby. */
	val = readl(DM250_VOP_BASE + DM250_VOP_SYS_CTRL);
	val &= ~DM250_VOP_WIN0_EN;
	val |= DM250_VOP_STANDBY;
	writel(val, DM250_VOP_BASE + DM250_VOP_SYS_CTRL);
	writel(1, DM250_VOP_BASE + DM250_VOP_REG_CFG_DONE);

	/* Allow the pending shadow-register update to reach a frame boundary. */
	mdelay(20);

	val = readl(DM250_VOP_BASE + DM250_VOP_AXI_BUS_CTRL);
	val &= ~DM250_VOP_OUTPUT_CLK_EN;
	writel(val, DM250_VOP_BASE + DM250_VOP_AXI_BUS_CTRL);

	ret = uclass_get_device_by_driver(UCLASS_DISPLAY,
					  DM_DRIVER_GET(rk3126_lvds), &display);
	if (!ret) {
		ret = generic_phy_get_by_name(display, "dphy", &dphy);
		if (!ret)
			have_dphy = true;
		else
			printf("DM250: LVDS PHY unavailable during shutdown (%d)\n",
			       ret);
	} else {
		printf("DM250: LVDS display unavailable during shutdown (%d)\n",
		       ret);
	}

	/* GRF writes use the upper half-word as a write mask. */
	writel(DM250_LVDS_CON0_WMSK << 16, DM250_GRF_LVDS_CON0);
	if (have_dphy) {
		ret = generic_phy_power_off(&dphy);
		if (ret)
			printf("DM250: failed to power off LVDS PHY (%d)\n", ret);
	}

	puts("DM250: display disabled before legacy U-Boot\n");
}

static int do_pomera_boot(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	const struct dm250_layout *layout = &sd_layout;
	const char *source = cmdtp->name;
	struct udevice *dev;
	struct blk_desc *desc;
	struct mmc *mmc;
	u32 kernel_size;
	int seq = 1, ret;
	bool chainload_legacy = false;
	unsigned long legacy_rc;
	char command[96];

	if (argc != 1)
		return CMD_RET_USAGE;

	if (!strcmp(source, "emmc")) {
		seq = 0;
		chainload_legacy = true;
	} else if (strcmp(source, "sd")) {
		return CMD_RET_USAGE;
	}
	if (chainload_legacy)
		printf("DM250: boot from %s (mmc %d), legacy U-Boot\n",
		       source, seq);
	else
		printf("DM250: boot from %s (mmc %d), direct rootfs\n",
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
		dm250_display_shutdown();
		puts("DM250: loading legacy eMMC U-Boot\n");
		ret = load_legacy_uboot(desc);
		if (ret)
			goto fail;

		/* The legacy image is ARM code; do not use the go command here. */
		legacy_rc = ((unsigned long (*)(int, char *const []))
				(ulong)LEGACY_UBOOT_ADDR)(0, NULL);
		printf("DM250: legacy eMMC U-Boot returned (0x%lx)\n",
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
	printf("DM250: kernel %u bytes, separate resource DTB, no initramfs\n",
	       kernel_size);

	/* '-' tells bootz that no external initramfs is present. */
	snprintf(command, sizeof(command), "bootz %x - %x",
		 KERNEL_ADDR, FDT_ADDR);
	return run_command(command, 0) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;

fail:
	printf("DM250: image load failed (%d)\n", ret);
	return CMD_RET_FAILURE;
}

U_BOOT_CMD(sd, 1, 0, do_pomera_boot,
	   "boot DM250 directly from SD", "");
U_BOOT_CMD(emmc, 1, 0, do_pomera_boot,
	   "chainload the legacy DM250 eMMC U-Boot", "");
