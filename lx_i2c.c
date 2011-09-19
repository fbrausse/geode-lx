
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <drm/drm_edid.h>

#include "drmP.h"

#include "lx.h"

#define GPIO_SCL 3
#define GPIO_SDA 4

static void lx_ddc_setsda(void *data, int state) {
	gpio_set_value(GPIO_SDA, state);
}

static void lx_ddc_setscl(void *data, int state) {
	gpio_set_value(GPIO_SCL, state);
}

static int lx_ddc_getsda(void *data) {
	return gpio_get_value(GPIO_SDA);
}

static int lx_ddc_getscl(void *data) {
	return gpio_get_value(GPIO_SCL);
}

static struct i2c_algo_bit_data lx_ddc_bit_data = {
	.data = NULL,
	.setsda = lx_ddc_setsda,
	.setscl = lx_ddc_setscl,
	.getsda = lx_ddc_getsda,
	.getscl = lx_ddc_getscl,
	.udelay = 5,			/* 100 kHz */
	.timeout = DRM_HZ / 10,		/* 100 ms */
};

/* copy of the non-exported function drm_probe_ddc() */
bool lx_ddc_probe(struct i2c_adapter *ddc) {
	int retries = 5;
	u8 start = 0, out, ret;

	do {
		struct i2c_msg msgs[] = {
			{
				.addr	= DDC_ADDR,
				.flags	= 0,
				.len	= 1,
				.buf	= &start,
			}, {
				.addr	= DDC_ADDR,
				.flags	= I2C_M_RD,
				.len	= 1,
				.buf	= &out,
			}
		};
		ret = i2c_transfer(ddc, msgs, ARRAY_SIZE(msgs));
		if (ret == ARRAY_SIZE(msgs))
			return true;
	} while (--retries);

	return false;
}

int lx_ddc_init(struct drm_device *dev) {
	struct i2c_adapter *adap;
	struct lx_priv *priv = dev->dev_private;
	int ret = -ENOMEM;

	adap = kzalloc(sizeof(*adap), GFP_KERNEL);
	if (!adap)
		goto failed_alloc_adap;

	ret = gpio_request(GPIO_SDA, "ddc-sda");
	if (ret)
		goto failed_sda;
	ret = gpio_request(GPIO_SCL, "ddc-scl");
	if (ret)
		goto failed_scl;

	gpio_direction_output(GPIO_SDA, 1);
	gpio_direction_output(GPIO_SCL, 1);

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "lx-ddc");
	adap->algo_data = &lx_ddc_bit_data;
	adap->class = I2C_CLASS_DDC;
	adap->dev.parent = dev->dev;

	ret = i2c_bit_add_bus(adap);
	if (ret)
		goto failed_add_bus;

	priv->ddc = adap;
	DRM_INFO("lx: added DDC adapter (id: %d)\n", adap->nr);
	return 0;

failed_add_bus:
	gpio_free(GPIO_SCL);
failed_scl:
	gpio_free(GPIO_SDA);
failed_sda:
	kfree(adap);
failed_alloc_adap:
	return ret;
}

void lx_ddc_cleanup(struct drm_device *dev) {
	struct lx_priv *priv = dev->dev_private;
	struct i2c_adapter *ddc = priv->ddc;

	if (!ddc)
		return;

	i2c_del_adapter(ddc);
	gpio_free(GPIO_SCL);
	gpio_free(GPIO_SDA);
	kfree(ddc);

	priv->ddc = NULL;
}
