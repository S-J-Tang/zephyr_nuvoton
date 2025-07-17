/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm4xx_spip

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spip_npcm4xx);

#include <drivers/clock_control.h>
#include <dt-bindings/clock/npcm4xx_clock.h>
#include <drivers/spi.h>
#include <soc.h>
#include "spi_context.h"

#define SPI_NPCM_SPIP_SINGLE                 0x0
#define SPI_NPCM_SPIP_DUAL                   0x1
#define SPI_NPCM_SPIP_QUAD                   0x2

/* Transfer this NOP value when tx buf is null */
#define SPI_NPCM_SPIP_TX_NOP                 0x00
#define SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US 1000

#if defined(CONFIG_ARCH_POSIX)
#define Z_SPIN_DELAY(t) k_busy_wait(t)
#else
#define Z_SPIN_DELAY(t)
#endif


#define WAIT_FOR(expr, timeout, delay_stmt)                                                        \
	({                                                                                         \
		uint32_t _wf_cycle_count = k_us_to_cyc_ceil32(timeout);                            \
		uint32_t _wf_start = k_cycle_get_32();                                             \
		while (!(expr) && (_wf_cycle_count > (k_cycle_get_32() - _wf_start))) {            \
			delay_stmt;                                                                \
			Z_SPIN_DELAY(10);                                                          \
		}                                                                                  \
		(expr);                                                                            \
	})

/* Device constant configuration parameters */
struct npcm4xx_spip_config {
	/* SPIP reg base address */
	uintptr_t base;
	/* clock configuration */
	struct npcm4xx_clk_cfg clk_cfg;
	struct npcm4xx_clk_cfg clk_cfg_control;
};

/* Device run time data */
struct npcm4xx_spip_data {
	struct spi_context ctx;
	uint32_t mclk;
	uint32_t apb3;
	/* read/write init flags */
	int rw_init;
	/* read command data */
	struct spi_nor_op_info read_op_info;
	/* write command data */
	struct spi_nor_op_info write_op_info;
	uint8_t bytes_per_frame;
	uint8_t access_mode;
	/* CS control */
	bool cs_sw;
};

/* Driver convenience defines */
#define HAL_INSTANCE(dev)									\
	((struct spip_reg *)((const struct npcm4xx_spip_config *)(dev)->config)->base)

static inline bool spi_npcm_spip_is_cs_gpio73(const struct device *dev,
						  const struct spi_config *spi_cfg)
{
	/* Check gpio pin is set as GPIO73 */
	if (spi_cfg->cs && spi_cfg->cs->gpio_dev && spi_cfg->cs->gpio_pin == 3 &&
	    spi_cfg->cs->gpio_dev == DEVICE_DT_GET(DT_NODELABEL(gpio7))) {
		return true;
	}

	return false;
}

static inline void spi_npcm_spip_cs_control(const struct device *dev,
					    const struct spi_config *spi_cfg, bool on)
{
	struct spip_reg *const inst = HAL_INSTANCE(dev);
	struct npcm4xx_spip_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	/*
	 * The GPIO73/SPIP1_CS is bundled with SPIP1_CLK, SPIP1_MOSI, SPIP1_DIO1,
	 * it cannot be set individually. So if the software CS is GPIO73, we should
	 * use hardware CS control.
	 */
	if ((data->cs_sw) && (ctx->config->cs && ctx->config->cs->gpio_dev) &&
	    ((!spi_npcm_spip_is_cs_gpio73(dev, spi_cfg)))) {
		/* SW CS pin */
		LOG_DBG("sw cs control, %s pin %d",
			ctx->config->cs->gpio_dev->name, ctx->config->cs->gpio_pin);
		spi_context_cs_control(ctx, on);
	} else {
		/* HW CS pin */
		LOG_DBG("hw cs control");
		if (on) {
			/* Set active, active high/low depend on SSACTPOL */
			inst->SSCTL |= BIT(NPCM4XX_SSCTL_SS);
		} else {
			if (spi_cfg->operation & SPI_HOLD_ON_CS) {
				return;
			}
			inst->SSCTL &= ~BIT(NPCM4XX_SSCTL_SS);
		}
	}
}

static int spip_npcm4xx_configure(const struct device *dev,
				  const struct spi_config *config)
{
	struct npcm4xx_spip_data *data = dev->data;
	struct spip_reg *const inst = HAL_INSTANCE(dev);
	uint32_t u32Div = 0;
	uint32_t target_freq;
	uint32_t source_clk;
	int ret = 0;
	uint16_t operation = config->operation;
	uint8_t frame_size;

	if (SPI_OP_MODE_GET(operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Only SPI controller mode is supported");
		return -ENOTSUP;
	}

	if (operation & SPI_MODE_LOOP) {
		LOG_ERR("Loopback mode is not supported");
		return -ENOTSUP;
	}
	data->ctx.config = config;

	/* Get the frame length */
	frame_size = SPI_WORD_SIZE_GET(operation);

	switch (frame_size) {
		case 8:
			data->bytes_per_frame = 1;
			break;
		case 16:
			data->bytes_per_frame = 2;
			break;
		case 24:
			data->bytes_per_frame = 3;
			break;
		case 32:
			data->bytes_per_frame = 4;
			break;
		default:
			LOG_ERR("Only support word sizes 8/16/32 bits");
			return -ENOTSUP;
	}

	if (frame_size == 32) {
		SET_FIELD(inst->CTL, NPCM4XX_CTL_DWIDTH, 0x0);
	} else {
		SET_FIELD(inst->CTL, NPCM4XX_CTL_DWIDTH, frame_size);
	}

	switch (operation & SPI_LINES_MASK) {
		case SPI_LINES_SINGLE:
			inst->CTL &= ~BIT(NPCM4XX_CTL_DUALIOEN);
			inst->CTL &= ~BIT(NPCM4XX_CTL_QUADIOEN);
			data->access_mode = SPI_NPCM_SPIP_SINGLE;
			break;
		case SPI_LINES_DUAL:
			inst->CTL |= BIT(NPCM4XX_CTL_DUALIOEN);
			inst->CTL &= ~BIT(NPCM4XX_CTL_QUADIOEN);
			data->access_mode = SPI_NPCM_SPIP_DUAL;
			break;
		case SPI_LINES_QUAD:
			inst->CTL &= ~BIT(NPCM4XX_CTL_DUALIOEN);
			inst->CTL |= BIT(NPCM4XX_CTL_QUADIOEN);
			data->access_mode = SPI_NPCM_SPIP_QUAD;
			break;
		default:
			LOG_ERR("Only single/dual/quad line mode is supported");
			return -ENOTSUP;
	}

	/* Set the endianness */
	if (operation & SPI_TRANSFER_LSB) {
		inst->CTL |= BIT(NPCM4XX_CTL_LSB);
	} else {
		inst->CTL &= ~BIT(NPCM4XX_CTL_LSB);
	}

	/*
	 * Set CPOL and CPHA.
	 * The following is how to map npcm spip control register to CPOL and CPHA
	 *   CPOL    CPHA  |  CLKPOL  TXNEG   RXNEG
	 *   --------------------------------------
	 *    0       0    |    0       1       0
	 *    0       1    |    0       0       1
	 *    1       0    |    1       0       1
	 *    1       1    |    1       1       0
	 */
	if (SPI_MODE_GET(operation) & SPI_MODE_CPOL) {
		inst->CTL |= BIT(NPCM4XX_CTL_CLKPOL);
		if (SPI_MODE_GET(operation) & SPI_MODE_CPHA) {
			inst->CTL |= BIT(NPCM4XX_CTL_TXNEG);
			inst->CTL &= ~BIT(NPCM4XX_CTL_RXNEG);
		} else {
			inst->CTL &= ~BIT(NPCM4XX_CTL_TXNEG);
			inst->CTL |= BIT(NPCM4XX_CTL_RXNEG);
		}
	} else {
		inst->CTL &= ~BIT(NPCM4XX_CTL_CLKPOL);
		if (SPI_MODE_GET(operation) & SPI_MODE_CPHA) {
			inst->CTL &= ~BIT(NPCM4XX_CTL_TXNEG);
			inst->CTL |= BIT(NPCM4XX_CTL_RXNEG);
		} else {
			inst->CTL |= BIT(NPCM4XX_CTL_TXNEG);
			inst->CTL &= ~BIT(NPCM4XX_CTL_RXNEG);
		}
	}

	/* Set CS */
	if ((data->cs_sw) && (!spi_npcm_spip_is_cs_gpio73(dev, config))) {
		spi_context_cs_configure(&data->ctx);
	}

	/* Active high CS logic */
	if (operation & SPI_CS_ACTIVE_HIGH) {
		inst->SSCTL |= BIT(NPCM4XX_SSCTL_SSACTPOL);
	} else {
		inst->SSCTL &= ~BIT(NPCM4XX_SSCTL_SSACTPOL);
	}

	/* Disable AUTOSS */
	inst->SSCTL &= ~BIT(NPCM4XX_SSCTL_AUTOSS);

	/* Set Bus clock */
	if (config->frequency != 0) {
		target_freq = config->frequency;
		source_clk = data->mclk;

		/* Clamp frequency to maximum supported by APB3 */
		if (target_freq > data->apb3) {
			LOG_WRN("Frequency %d is larger than APB3 clock %d, clamping to APB3",
				target_freq, data->apb3);
			target_freq = data->apb3;
		}

		/* Calculate divider with proper rounding */
		u32Div = (source_clk + target_freq - 1) / target_freq - 1;

		/* Clamp divider to maximum supported value */
		if (u32Div > 0xF) {
			LOG_WRN("Divider %d is larger than maximum 15, clamping to 15", u32Div);
			u32Div = 0xF;
		}
	}

	inst->CLKDIV = (inst->CLKDIV & ~0xF) | u32Div;

	return ret;
}

static bool spi_npcm_spip_transfer_ongoing(struct npcm4xx_spip_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static int spi_npcm_spip_process_tx_buf(struct npcm4xx_spip_data *const data, uint32_t *tx_frame)
{
	int ret = 0;

	/* Get the tx_frame from tx_buf only when tx_buf != NULL */
	if (spi_context_tx_buf_on(&data->ctx)) {
		*tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
	} else {
		ret = -ENOBUFS;
	}

	/*
	 * The update is ignored if TX is off (tx_len == 0).
	 * Note: if tx_buf == NULL && tx_len != 0, the update still counts.
	 */
	spi_context_update_tx(&data->ctx, data->bytes_per_frame, 1);

	return ret;
}

static void spi_npcm_spip_process_rx_buf(struct npcm4xx_spip_data *const data, uint32_t rx_frame)
{
	if (spi_context_rx_buf_on(&data->ctx)) {
		UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
	}

	spi_context_update_rx(&data->ctx, data->bytes_per_frame, 1);
}

static int spi_npcm_spip_xfer_frame(const struct device *dev)
{
	struct npcm4xx_spip_data *data = dev->data;
	struct spip_reg *const inst = HAL_INSTANCE(dev);
	uint32_t tx_frame = SPI_NPCM_SPIP_TX_NOP;
	uint32_t rx_frame;
	int ret;

	if (WAIT_FOR(!IS_BIT_SET(inst->STATUS, NPCM4XX_STATUS_TXFULL),
		     SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
		LOG_ERR("Wait for TX empty Timeout");
		return -ETIMEDOUT;
	}

	ret = spi_npcm_spip_process_tx_buf(data, &tx_frame);

	if (WAIT_FOR(!IS_BIT_SET(inst->STATUS, NPCM4XX_STATUS_BUSY),
		     SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
		LOG_ERR("Check Status BUSY Timeout");
		return -ETIMEDOUT;
	}

	if (data->access_mode != SPI_NPCM_SPIP_SINGLE) {
		if (ret == -ENOBUFS) {
			/* Input mode */
			inst->CTL &= ~BIT(NPCM4XX_CTL_QDIODIR);
		} else {
			/* Output mode */
			inst->CTL |= BIT(NPCM4XX_CTL_QDIODIR);
		}
	}

	inst->TX = tx_frame;

	if (WAIT_FOR(!IS_BIT_SET(inst->STATUS, NPCM4XX_STATUS_RXEMPTY),
		     SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
		LOG_ERR("Check Status RBF Timeout");
		return -ETIMEDOUT;
	}

	rx_frame = inst->RX;
	spi_npcm_spip_process_rx_buf(data, rx_frame);

	return 0;
}

static int spip_npcm4xx_transceive(const struct device *dev,
				   const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs)
{
	struct spip_reg *const inst = HAL_INSTANCE(dev);
	struct npcm4xx_spip_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0;

	spi_context_lock(ctx, false, NULL, config);
	ctx->config = config;

	/* Configure */
	ret = spip_npcm4xx_configure(dev, config);
	if (ret) {
		spi_context_release(ctx, ret);
		return ret;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);
	if (!spi_npcm_spip_transfer_ongoing(data)) {
		spi_context_release(ctx, 0);
		return 0;
	}

	/* Cleaning junk data in the buffer */
	while (!IS_BIT_SET(inst->STATUS, NPCM4XX_STATUS_RXEMPTY)) {
		uint8_t unused __attribute__((unused));

		unused = inst->RX;
	}

	/* spip enable */
	inst->FIFOCTL |= BIT(NPCM4XX_FIFOCTL_RXRST);
	inst->FIFOCTL |= BIT(NPCM4XX_FIFOCTL_TXRST);
	inst->CTL |= BIT(NPCM4XX_CTL_SPIEN);

	/* set active true */
	spi_npcm_spip_cs_control(dev, config, true);

	do {
		ret = spi_npcm_spip_xfer_frame(dev);
		if (ret < 0)
			break;
	} while (spi_npcm_spip_transfer_ongoing(data));

	/* set active false */
	spi_npcm_spip_cs_control(dev, config, false);

	/* spip disable */
	inst->CTL &= ~BIT(NPCM4XX_CTL_SPIEN);
	spi_context_release(ctx, ret);

	return ret;
}

#ifdef CONFIG_SPI_ASYNC
static int spip_npcm4xx_transceive_async(const struct device *dev,
					 const struct spi_config *config,
					 const struct spi_buf_set *tx_bufs,
					 const struct spi_buf_set *rx_bufs)
{
	return spip_npcm4xx_transceive(dev, config, tx_bufs, rx_bufs);
}
#endif /* CONFIG_SPI_ASYNC */

static int spip_npcm4xx_release(const struct device *dev,
				const struct spi_config *config)
{
	struct npcm4xx_spip_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spip_npcm4xx_init(const struct device *dev)
{
	const struct npcm4xx_spip_config *cfg = dev->config;
	struct npcm4xx_spip_data *data = dev->data;
	const struct device *const clk_dev =
		device_get_binding(NPCM4XX_CLK_CTRL_NAME);
	uint32_t mclk;
	uint32_t apb3;
	int ret;

	if (!device_is_ready(clk_dev)) {
		LOG_ERR("%s device not ready", clk_dev->name);
		return -ENODEV;
	}
	/* Turn on device clock first and get source clock freq. */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t)&cfg->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on SPIP clock fail %d", ret);
		return ret;
	}

	/* Get MCLK */
	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t *)&cfg->clk_cfg, &mclk);
	if (ret < 0) {
		LOG_ERR("Get MCLK clock rate error %d", ret);
		return ret;
	}
	data->mclk = mclk;

	/* Get APB3 as control source */
	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t *)&cfg->clk_cfg_control, &apb3);
	if (ret < 0) {
		LOG_ERR("Get APB3 clock rate error %d", ret);
		return ret;
	}
	data->apb3 = apb3;

	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static inline void spi_npcm4xx_spip_write_data(const struct device *dev, uint32_t code)
{
	struct spip_reg *const inst = HAL_INSTANCE(dev);

	while ((inst->STATUS & BIT(NPCM4XX_STATUS_TXFULL)));

	inst->TX = code;

	while (inst->STATUS & BIT(NPCM4XX_STATUS_BUSY));
}

static void spi_nor_npcm4xx_spip_fifo_transceive(const struct device *dev,
					const struct spi_config *spi_cfg,
					struct spi_nor_op_info op_info)
{
	struct spip_reg *const inst = HAL_INSTANCE(dev);
	struct spi_nor_op_info *normal_op_info = NULL;
	uint32_t index = 0, dummy_write = 0;
	uint8_t *buf_data = NULL;
	uint8_t sub_addr = 0;

	normal_op_info = &op_info;

	inst->CTL |= BIT(NPCM4XX_CTL_SPIEN);

	/* clear tx/rx fifo buffer */
	inst->FIFOCTL |= BIT(NPCM4XX_FIFOCTL_TXRST);
	inst->FIFOCTL |= BIT(NPCM4XX_FIFOCTL_RXRST);

	/* set active true */
	spi_npcm_spip_cs_control(dev, spi_cfg, true);

	/* send command */
	spi_npcm4xx_spip_write_data(dev, (uint32_t)normal_op_info->opcode);

	/* send address */
	index = normal_op_info->addr_len;
	while (index) {
		index = index - 1;
		sub_addr = (normal_op_info->addr >> (8 * index)) & 0xff;
		spi_npcm4xx_spip_write_data(dev, (uint32_t)sub_addr);
	}

	/* only support single mode dummy byte */
	if ((normal_op_info->dummy_cycle % NPCM4XX_SPIP_SINGLE_DUMMY_BYTE) != 0) {
		LOG_ERR("SPIP now only support single mode");
		return;
	}

	/* send dummy bytes */
	for (index = 0; index < normal_op_info->dummy_cycle;
				index += NPCM4XX_SPIP_SINGLE_DUMMY_BYTE) {
		spi_npcm4xx_spip_write_data(dev, dummy_write);
	}

	/* clear rx fifo buffer */
	inst->FIFOCTL |= BIT(NPCM4XX_FIFOCTL_RXRST);

	buf_data = normal_op_info->buf;

	if (buf_data == NULL) {
		inst->FIFOCTL |= BIT(NPCM4XX_FIFOCTL_RXRST);
		goto spi_nor_normal_done;
	}

	/* read data from SPI flash */
	if (normal_op_info->data_direct == SPI_NOR_DATA_DIRECT_IN) {
		for (index = 0; index < normal_op_info->data_len; index++) {
			/* write dummy data out*/
			spi_npcm4xx_spip_write_data(dev, dummy_write);
			/* wait received data */
			if (WAIT_FOR(!IS_BIT_SET(inst->STATUS, NPCM4XX_STATUS_RXEMPTY),
				     SPI_NPCM_SPIP_WAIT_STATUS_TIMEOUT_US, NULL) == false) {
				LOG_ERR("Wait for RX data timeout");
				goto spi_nor_normal_done;
			}

			*(buf_data + index) = (uint8_t)inst->RX;
		}
	} else if (normal_op_info->data_direct == SPI_NOR_DATA_DIRECT_OUT) {
		for (index = 0; index < normal_op_info->data_len; index++) {
			/* write data to SPI flash */
			spi_npcm4xx_spip_write_data(dev, (uint32_t)*(buf_data + index));
		}
		/* clear rx fifo buffer */
		inst->FIFOCTL |= BIT(NPCM4XX_FIFOCTL_RXRST);
	}

spi_nor_normal_done:

	/* set active false */
	spi_npcm_spip_cs_control(dev, spi_cfg, false);

	/* spip disable */
	inst->CTL &= ~BIT(NPCM4XX_CTL_SPIEN);
}

static int spi_nor_npcm4xx_spip_transceive(const struct device *dev,
					const struct spi_config *spi_cfg,
					struct spi_nor_op_info op_info)
{
	struct npcm4xx_spip_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret = 0, error = 0;

	ret = spip_npcm4xx_configure(dev, spi_cfg);
	if (ret)
		return ret;

	spi_context_lock(ctx, false, NULL, spi_cfg);
	ctx->config = spi_cfg;

	spi_nor_npcm4xx_spip_fifo_transceive(dev, spi_cfg, op_info);

	spi_context_release(ctx, error);

	return error;
}

static int spi_nor_npcm4xx_spip_read_init(const struct device *dev,
					const struct spi_config *spi_cfg,
					struct spi_nor_op_info op_info)
{
	struct npcm4xx_spip_data *data = dev->data;

	/* record read command from jesd216 */
	memcpy(&data->read_op_info, &op_info, sizeof(op_info));

	data->rw_init |= NPCM4XX_SPIP_SPI_NOR_READ_INIT_OK;

	return 0;
}

static int spi_nor_npcm4xx_spip_write_init(const struct device *dev,
					const struct spi_config *spi_cfg,
					struct spi_nor_op_info op_info)
{
	struct npcm4xx_spip_data *data = dev->data;

	/* record read command from jesd216 */
	memcpy(&data->write_op_info, &op_info, sizeof(op_info));

	data->rw_init |= NPCM4XX_SPIP_SPI_NOR_WRITE_INIT_OK;

	return 0;
}

static const struct spi_nor_ops npcm4xx_spip_spi_nor_ops = {
	.transceive = spi_nor_npcm4xx_spip_transceive,
	.read_init = spi_nor_npcm4xx_spip_read_init,
	.write_init = spi_nor_npcm4xx_spip_write_init,
};

static const struct spi_driver_api spip_npcm4xx_driver_api = {
	.transceive = spip_npcm4xx_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spip_npcm4xx_transceive_async,
#endif
	.release = spip_npcm4xx_release,
	.spi_nor_op = &npcm4xx_spip_spi_nor_ops,
};


static const struct npcm4xx_spip_config spip_npcm4xx_config = {
	.base = DT_INST_REG_ADDR(0),
	.clk_cfg = NPCM4XX_DT_CLK_CFG_ITEM_BY_IDX(0, 0) // mclk
	.clk_cfg_control = NPCM4XX_DT_CLK_CFG_ITEM_BY_IDX(0, 1) // apb3
};

static struct npcm4xx_spip_data spip_npcm4xx_dev_data = {
	SPI_CONTEXT_INIT_LOCK(spip_npcm4xx_dev_data, ctx),
	.cs_sw = DT_INST_PROP(0, cs_sw),
};


DEVICE_DT_INST_DEFINE(0, &spip_npcm4xx_init, NULL,
		      &spip_npcm4xx_dev_data,
		      &spip_npcm4xx_config, POST_KERNEL,
		      CONFIG_SPI_INIT_PRIORITY, &spip_npcm4xx_driver_api);
