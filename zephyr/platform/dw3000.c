#include <device.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <zephyr.h>

#include "deca_device_api.h"
#include "dw3000.h"

LOG_MODULE_REGISTER(dw3000);

#define DW_INST DT_INST(0, decawave_dw3000)

static struct gpio_callback gpio_cb;
static struct k_work dw3000_isr_work;

struct dw3000_config {
	struct gpio_dt_spec gpio_irq;
	struct gpio_dt_spec gpio_reset;
	struct gpio_dt_spec gpio_wakeup;
	struct gpio_dt_spec gpio_spi_pol;
	struct gpio_dt_spec gpio_spi_pha;
};

static const struct dw3000_config conf = {
	.gpio_irq = GPIO_DT_SPEC_GET_OR(DW_INST, irq_gpios, {0}),
	.gpio_reset = GPIO_DT_SPEC_GET_OR(DW_INST, reset_gpios, {0}),
	.gpio_wakeup = GPIO_DT_SPEC_GET_OR(DW_INST, wakeup_gpios, {0}),
	.gpio_spi_pol = GPIO_DT_SPEC_GET_OR(DW_INST, spi_pol_gpios, {0}),
	.gpio_spi_pha = GPIO_DT_SPEC_GET_OR(DW_INST, spi_pha_gpios, {0}),
};

/* deca_spi.c */
int dw3000_spi_init(void);
void dw3000_spi_fini(void);
void dw3000_spi_wakeup(void);

int dw3000_init()
{
	/* Reset */
	if (conf.gpio_reset.port) {
		gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
		LOG_INF("RESET on %s pin %d", conf.gpio_reset.port->name,
				conf.gpio_reset.pin);
	}

	/* Wakeup (optional) */
	if (conf.gpio_wakeup.port) {
		gpio_pin_configure_dt(&conf.gpio_wakeup, GPIO_OUTPUT_ACTIVE);
		LOG_INF("WAKEUP on %s pin %d", conf.gpio_wakeup.port->name,
				conf.gpio_wakeup.pin);
	}

	/* SPI Polarity (optional) */
	if (conf.gpio_spi_pol.port) {
		gpio_pin_configure_dt(&conf.gpio_spi_pol, GPIO_OUTPUT_INACTIVE);
		LOG_INF("SPI_POL on %s pin %d", conf.gpio_spi_pol.port->name,
				conf.gpio_spi_pol.pin);
	}

	/* SPI Phase (optional) */
	if (conf.gpio_spi_pha.port) {
		gpio_pin_configure_dt(&conf.gpio_spi_pha, GPIO_OUTPUT_INACTIVE);
		LOG_INF("SPI_PHA on %s pin %d", conf.gpio_spi_pha.port->name,
				conf.gpio_spi_pha.pin);
	}

	return dw3000_spi_init();
}

static void dw3000_isr_work_handler(struct k_work* item)
{
	dwt_isr();
}

static void dw3000_isr(const struct device* dev, struct gpio_callback* cb,
					   uint32_t pins)
{
	k_work_submit(&dw3000_isr_work);
}

int dw3000_init_interrupt(void)
{
	k_work_init(&dw3000_isr_work, dw3000_isr_work_handler);

	/* Interrupt */
	if (conf.gpio_irq.port) {
		gpio_pin_configure_dt(&conf.gpio_irq, GPIO_INPUT);
		gpio_init_callback(&gpio_cb, dw3000_isr, BIT(conf.gpio_irq.pin));
		gpio_add_callback(conf.gpio_irq.port, &gpio_cb);
		gpio_pin_interrupt_configure_dt(&conf.gpio_irq, GPIO_INT_EDGE_RISING);

		LOG_INF("IRQ on %s pin %d", conf.gpio_irq.port->name,
				conf.gpio_irq.pin);
		return 0;
	} else {
		return -ENOENT;
	}
}

void dw3000_fini(void)
{
	// TODO
	dw3000_spi_fini();
}

void dw3000_hw_reset()
{
	if (!conf.gpio_reset.port) {
		LOG_ERR("No HW reset configured");
		return;
	}

	gpio_pin_configure_dt(&conf.gpio_reset, GPIO_OUTPUT_ACTIVE);
	k_msleep(1); // 10 us?
	gpio_pin_configure_dt(&conf.gpio_reset, GPIO_INPUT);
	k_msleep(2);
}

/** wakeup either using the WAKEUP pin or SPI CS */
void dw3000_wakeup(void)
{
	if (conf.gpio_wakeup.port) {
		/* Use WAKEUP pin if available */
		LOG_INF("WAKEUP PIN");
		gpio_pin_set_dt(&conf.gpio_wakeup, 1);
	} else {
		/* Use SPI CS pin */
		LOG_INF("WAKEUP CS");
		dw3000_spi_wakeup();
	}
	k_sleep(K_MSEC(1));
}

/** set WAKEUP pin low if available */
void dw3000_wakeup_pin_low(void)
{
	if (conf.gpio_wakeup.port) {
		gpio_pin_set_dt(&conf.gpio_wakeup, 0);
	}
}
