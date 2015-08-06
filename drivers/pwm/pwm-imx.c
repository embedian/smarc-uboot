/*
 * (C) Copyright 2014
 * Heiko Schocher, DENX Software Engineering, hs at denx.de.
 *
 * Basic support for the pwm modul on imx6.
 *
 * Based on linux:drivers/pwm/pwm-imx.c
 * from
 * Sascha Hauer <s.hauer at pengutronix.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <div64.h>
#include <pwm.h>
#include <asm/arch/imx-regs.h>
#include <asm/io.h>

/* pwm_id from 0..3 */
static struct pwm_regs *pwm_id_to_reg(int pwm_id)
{
	switch (pwm_id) {
	case 0:
		return (struct pwm_regs *)PWM1_BASE_ADDR;
		break;
	case 1:
		return (struct pwm_regs *)PWM2_BASE_ADDR;
		break;
	case 2:
		return (struct pwm_regs *)PWM3_BASE_ADDR;
		break;
	case 3:
		return (struct pwm_regs *)PWM4_BASE_ADDR;
		break;
	default:
		printf("unknown pwm_id: %d\n", pwm_id);
		break;
	}
	return NULL;
}

int pwm_init(int pwm_id, int div, int invert)
{
	struct pwm_regs *pwm = (struct pwm_regs *)pwm_id_to_reg(pwm_id);

	writel(0, &pwm->ir);
	return 0;
}

int pwm_config(int pwm_id, int duty_ns, int period_ns)
{
	struct pwm_regs *pwm = (struct pwm_regs *)pwm_id_to_reg(pwm_id);
	unsigned long long c;
	unsigned long period_cycles, duty_cycles, prescale;
	u32 cr;

	/*
	 * we have not yet a clock framework for imx6, so add the clock
	 * value here as a define. Replace it when we have the clock
	 * framework.
	 */
	c = CONFIG_IMX6_PWM_PER_CLK;
	c = c * period_ns;
	do_div(c, 1000000000);
	period_cycles = c;

	prescale = period_cycles / 0x10000 + 1;

	period_cycles /= prescale;
	c = (unsigned long long)period_cycles * duty_ns;
	do_div(c, period_ns);
	duty_cycles = c;

	/*
	 * according to imx pwm RM, the real period value should be
	 * PERIOD value in PWMPR plus 2.
	 */
	if (period_cycles > 2)
		period_cycles -= 2;
	else
		period_cycles = 0;

	cr = PWMCR_PRESCALER(prescale) |
		PWMCR_DOZEEN | PWMCR_WAITEN |
		PWMCR_DBGEN | PWMCR_CLKSRC_IPG_HIGH;

	writel(cr, &pwm->cr);
	/* set duty cycles */
	writel(duty_cycles, &pwm->sar);
	/* set period cycles */
	writel(period_cycles, &pwm->pr);
	return 0;
}

int pwm_enable(int pwm_id)
{
	struct pwm_regs *pwm = (struct pwm_regs *)pwm_id_to_reg(pwm_id);

	setbits_le32(&pwm->cr, PWMCR_EN);
	return 0;
}

void pwm_disable(int pwm_id)
{
	struct pwm_regs *pwm = (struct pwm_regs *)pwm_id_to_reg(pwm_id);

	clrbits_le32(&pwm->cr, PWMCR_EN);
} 
