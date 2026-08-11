// SPDX-License-Identifier: GPL-2.0+
/* DM200 hardware setup, based on King Jim's U-Boot 2014.10 sources. */
#include <config.h>
#include <button.h>
#include <dm.h>
#include <env.h>
#include <init.h>
#include <asm/io.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/arch-rockchip/grf_rk3128.h>
#include <power/pmic.h>
#include <power/rk8xx_pmic.h>

int arch_cpu_init(void)
{
	/* Stock firmware uses timer0; the ARM generic timer needs timer5. */
	writel(0, 0x200440b0);
	writel(0xffffffff, 0x200440a0);
	writel(0xffffffff, 0x200440a4);
	writel(1, 0x200440b0);
	/* Select timer5 as the architected counter source (HIWORD mask). */
	writel(0x80000000, 0x20000134);
	asm volatile("mcr p15, 0, %0, c14, c0, 0" : : "r"(24000000));
	dsb();
	return 0;
}

void board_debug_uart_init(void)
{
	struct rk3128_grf *grf = (void *)0x20008000;

	rk_clrsetreg(&grf->gpio1b_iomux,
		     GPIO1B1_MASK | GPIO1B2_MASK,
		     GPIO1B1_UART1_SOUT << GPIO1B1_SHIFT |
		     GPIO1B2_UART1_SIN << GPIO1B2_SHIFT);
}

static bool pressed(const char *label)
{
	struct udevice *dev;

	return !button_get_by_label(label, &dev) &&
		button_get_state(dev) == BUTTON_ON;
}

int rk_board_late_init(void)
{
	struct udevice *pmic;
	int ret;

	/* Preserve the stock three-key recovery gesture, without WARP. */
	if (pressed("Right Shift") && pressed("Left Alt") && pressed("Power")) {
		env_set("dm200_recovery", "1");
		puts("DM200: recovery keys held (used by dm200boot emmc)\n");
	}

	ret = uclass_get_device_by_name(UCLASS_PMIC, "pmic@1c", &pmic);
	if (ret) {
		printf("DM200: RK818 unavailable (%d)\n", ret);
		return 0;
	}
	/* pmic_rk818.c: rk818_pre_init(), keep the original register values. */
	ret = pmic_clrsetbits(pmic, 0xa1, 0, 0x70);
	if (ret)
		goto pmic_error;
	ret = pmic_clrsetbits(pmic, 0x52, 0, 0x02);
	if (ret)
		goto pmic_error;
	ret = pmic_clrsetbits(pmic, REG_DCDC_EN, 0, 0x60);
	if (ret)
		goto pmic_error;
	ret = pmic_reg_write(pmic, REG_CLK32OUT, 0x01);
	if (ret)
		goto pmic_error;
	return 0;

pmic_error:
	printf("DM200: RK818 setup failed (%d)\n", ret);
	return 0;
}
