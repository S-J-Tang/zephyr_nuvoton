/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_pwm

#include <drivers/pwm.h>
#include <dt-bindings/clock/npcm4xx_clock.h>
#include <drivers/clock_control.h>
#include <kernel.h>
#include <soc.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(pwm_npcm, LOG_LEVEL_ERR);

/* NPCM clock control node reference */
#define NPCM4XX_CLK_CTRL_NODE DT_NODELABEL(pcc)

/* PWM clock sources */
#define NPCM_PWM_CLOCK_APB2_LFCLK   0
#define NPCM_PWM_CLOCK_FX           1
#define NPCM_PWM_CLOCK_FR           2
#define NPCM_PWM_CLOCK_RESERVED     3

/* PWM heart-beat mode selection */
#define NPCM_PWM_HBM_NORMAL         0

/* 16-bit period cycles/prescaler in NPCM PWM modules */
#define NPCM_PWM_MAX_PRESCALER      (1UL << (16))
#define NPCM_PWM_MAX_PERIOD_CYCLES  (1UL << (16))

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

/* Device config */
struct pwm_npcm_config {
	/* pwm controller base address */
	struct pwm_reg *base;
	/* clock configuration */
	struct npcm4xx_clk_cfg clk_cfg;
};

/* Driver data */
struct pwm_npcm_data {
	/* PWM cycles per second */
	uint32_t cycles_per_sec;
};

/* PWM local functions */
static int pwm_npcm_configure(const struct device *dev, int clk_bus)
{
	const struct pwm_npcm_config *config = dev->config;
	struct pwm_reg *inst = config->base;

	/* Disable PWM for module configuration first */
	inst->PWMCTL &= ~BIT(NPCM4XX_PWMCTL_PWR);

	/* Set default PWM polarity to normal */
	inst->PWMCTL &= ~BIT(NPCM4XX_PWMCTL_INVP);

	/* Turn off PWM heart-beat mode */
	SET_FIELD(inst->PWMCTL, NPCM4XX_PWMCTL_HB_DC_CTL_FIELD,
			NPCM_PWM_HBM_NORMAL);

	/* Select APB CLK/LFCLK clock sources to PWM module by default */
	SET_FIELD(inst->PWMCTLEX, NPCM4XX_PWMCTLEX_FCK_SEL_FIELD,
			NPCM_PWM_CLOCK_APB2_LFCLK);

	/* Select input clock source to LFCLK or APB2 , otherwise failure */
	if (clk_bus == NPCM4XX_CLOCK_BUS_LFCLK) {
		inst->PWMCTL |= BIT(NPCM4XX_PWMCTL_CKSEL);
	} else if (clk_bus == NPCM4XX_CLOCK_BUS_APB2) {
		inst->PWMCTL &= ~BIT(NPCM4XX_PWMCTL_CKSEL);
	} else {
		LOG_ERR("Support LFCLK and APB2 input clock source only");
		return -EINVAL;
	}

	return 0;
}

/* PWM api functions */
static int pwm_npcm_pin_set(const struct device *dev, uint32_t channel,
			    uint32_t period_cycles, uint32_t pulse_cycles,
			    pwm_flags_t flags)
{
	/* Single channel for each pwm device */
	ARG_UNUSED(channel);
	const struct pwm_npcm_config *config = dev->config;
	struct pwm_npcm_data *const data = dev->data;
	struct pwm_reg *inst = config->base;
	int prescaler;
	uint32_t ctl;
	uint32_t ctr;
	uint32_t dcr;
	uint32_t prsc;

	ctl = inst->PWMCTL | BIT(NPCM4XX_PWMCTL_PWR);

	/* Select PWM inverted polarity (ie. active-low pulse). */
	if (flags & PWM_POLARITY_INVERTED) {
		ctl |= BIT(NPCM4XX_PWMCTL_INVP);
	} else {
		ctl &= ~BIT(NPCM4XX_PWMCTL_INVP);
	}

	/* If pulse_cycles is 0, switch PWM off and return. */
	if (pulse_cycles == 0) {
		ctl &= ~BIT(NPCM4XX_PWMCTL_PWR);
		inst->PWMCTL = ctl;
		return 0;
	}

	/*
	 * Calculate PWM prescaler that let period_cycles map to
	 * maximum pwm period cycles and won't exceed it.
	 * Then prescaler = ceil (period_cycles / pwm_max_period_cycles)
	 */
	prescaler = DIV_ROUND_UP(period_cycles, NPCM_PWM_MAX_PERIOD_CYCLES);
	if (prescaler > NPCM_PWM_MAX_PRESCALER) {
		return -EINVAL;
	}

	/* Set PWM prescaler. */
	prsc = prescaler - 1;

	/* Set PWM period cycles. */
	ctr = (period_cycles / prescaler) - 1;

	/* Set PWM pulse cycles. */
	dcr = (pulse_cycles / prescaler) - 1;

	LOG_DBG("freq %d, pre %d, period %d, pulse %d",
		data->cycles_per_sec / period_cycles, prsc, ctr, dcr);

	/* Reconfigure only if necessary. */
	if (inst->PWMCTL != ctl || inst->PRSC != prsc || inst->CTR != ctr) {
		/* Disable PWM before configuring. */
		inst->PWMCTL &= ~BIT(NPCM4XX_PWMCTL_PWR);

		inst->PRSC = prsc;
		inst->CTR = ctr;
		inst->DCR = dcr;

		/* Enable PWM now. */
		inst->PWMCTL = ctl;

		return 0;
	}

	inst->DCR = dcr;

	return 0;
}

static int pwm_npcm_get_cycles_per_sec(const struct device *dev,
				       uint32_t channel, uint64_t *cycles)
{
	/* Single channel for each pwm device */
	ARG_UNUSED(channel);
	struct pwm_npcm_data *const data = dev->data;

	*cycles = data->cycles_per_sec;
	return 0;
}

/* PWM driver registration */
static const struct pwm_driver_api pwm_npcm_driver_api = {
	.pin_set = pwm_npcm_pin_set,
	.get_cycles_per_sec = pwm_npcm_get_cycles_per_sec
};

static int pwm_npcm_init(const struct device *dev)
{
	const struct pwm_npcm_config *const config = dev->config;
	struct pwm_npcm_data *const data = dev->data;
    struct pwm_reg *const inst = config->base;
	const struct device *const clk_dev = DEVICE_DT_GET(NPCM4XX_CLK_CTRL_NODE);
	int ret;

	/*
	 * NPCM PWM module mixes byte and word registers together. Make sure
	 * word reg access via structure won't break into two byte reg accesses
	 * unexpectedly by toolchains options or attributes. If so, stall here.
	 */
	NPCM4XX_REG_WORD_ACCESS_CHECK(inst->PRSC, 0xA55A);

	if (!device_is_ready(clk_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Turn on device clock first and get source clock freq. */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t)
							&config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on PWM clock fail %d", ret);
		return ret;
	}

	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)
			&config->clk_cfg, &data->cycles_per_sec);
	if (ret < 0) {
		LOG_ERR("Get PWM clock rate error %d", ret);
		return ret;
	}

	/* Configure PWM device initially */
	ret = pwm_npcm_configure(dev, config->clk_cfg.bus);
	if (ret < 0) {
		LOG_ERR("PWM configure failed (%d)", ret);
		return ret;
	}

	return 0;
}

#define NPCM_PWM_INIT(inst)                                                             \
	static const struct pwm_npcm_config pwm_npcm_cfg_##inst = {                     \
		.base = (struct pwm_reg *)DT_INST_REG_ADDR(inst),                       \
		.clk_cfg = NPCM4XX_DT_CLK_CFG_ITEM(inst),                               \
	};                                                                              \
									                \
	static struct pwm_npcm_data pwm_npcm_data_##inst;                               \
									                \
	DEVICE_DT_INST_DEFINE(inst,					                \
			    &pwm_npcm_init, NULL,			                \
			    &pwm_npcm_data_##inst, &pwm_npcm_cfg_##inst,                \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,           \
			    &pwm_npcm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NPCM_PWM_INIT)
