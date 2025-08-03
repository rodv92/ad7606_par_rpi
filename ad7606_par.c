// SPDX-License-Identifier: GPL-2.0
/*
 * AD7606 Parallel Interface ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Notice : Derivative Work
 *
 * Copyright 2025 SKYNEXT Rodrigo Verissimo EURL
 *
 * Raspberry Pi ad7606 iio implementation, with CS/RD pin support for reading all channels
 * Implementation of a loadable module
 * Integration of MMIO address space into reg property of the device tree
 * instead of board support code
 * Adding support for byte shifting to extract conversion results on GPIO BCM
 * BCM pins GPIO8 to GPIO23 as parallel interface pins, while maintaining memory aligment constrains (4 byte alignment, 4 byte read i/o)
 * 
 * Tested on Raspberry Pi Zero W 1.1
 * Added debug logging
 *
 */
//#define DEBUG

#include <linux/types.h>
#include <linux/property.h>
#include <linux/sizes.h>

#include <linux/io.h>

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>


#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/util_macros.h>

#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/backend.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>



#include "ad7606.h"
#include "ad7606_bus_iface.h"



/*
 * Scales are computed as 5000/32768 and 10000/32768 respectively,
 * so that when applied to the raw values they provide mV values
 */
static const unsigned int ad7606_scale_avail[2] = {
	152588, 305176
};


static const unsigned int ad7616_sw_scale_avail[3] = {
	76293, 152588, 305176
};

static const unsigned int ad7606_oversampling_avail[7] = {
	1, 2, 4, 8, 16, 32, 64,
};

static const unsigned int ad7616_oversampling_avail[8] = {
	1, 2, 4, 8, 16, 32, 64, 128,
};



static ssize_t ad7606_show_avail(char *buf, const unsigned int *vals,
	unsigned int n, bool micros)
{
size_t len = 0;
int i;

for (i = 0; i < n; i++) {
len += scnprintf(buf + len, PAGE_SIZE - len,
micros ? "0.%06u " : "%u ", vals[i]);
}
buf[len - 1] = '\n';

return len;
}

static ssize_t ad7606_oversampling_ratio_avail(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
struct iio_dev *indio_dev = dev_to_iio_dev(dev);
struct ad7606_state *st = iio_priv(indio_dev);

return ad7606_show_avail(buf, st->oversampling_avail,
st->num_os_ratios, false);
}

static ssize_t in_voltage_scale_available_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
struct iio_dev *indio_dev = dev_to_iio_dev(dev);
struct ad7606_state *st = iio_priv(indio_dev);

return ad7606_show_avail(buf, st->scale_avail, st->num_scales, true);
}

static IIO_DEVICE_ATTR(oversampling_ratio_available, 0444,
ad7606_oversampling_ratio_avail, NULL, 0);

static IIO_DEVICE_ATTR_RO(in_voltage_scale_available, 0);

static struct attribute *ad7606_attributes_os_and_range[] = {
&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
NULL,
};

static const struct attribute_group ad7606_attribute_group_os_and_range = {
.attrs = ad7606_attributes_os_and_range,
};

static struct attribute *ad7606_attributes_os[] = {
&iio_dev_attr_oversampling_ratio_available.dev_attr.attr,
NULL,
};

static const struct attribute_group ad7606_attribute_group_os = {
.attrs = ad7606_attributes_os,
};

static struct attribute *ad7606_attributes_range[] = {
&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
NULL,
};

static const struct attribute_group ad7606_attribute_group_range = {
.attrs = ad7606_attributes_range,
};

static const struct iio_chan_spec ad7605_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(4),
	AD7605_CHANNEL(0),
	AD7605_CHANNEL(1),
	AD7605_CHANNEL(2),
	AD7605_CHANNEL(3),
};

static const struct iio_chan_spec ad7606_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(8),
	AD7606_CHANNEL(0),
	AD7606_CHANNEL(1),
	AD7606_CHANNEL(2),
	AD7606_CHANNEL(3),
	AD7606_CHANNEL(4),
	AD7606_CHANNEL(5),
	AD7606_CHANNEL(6),
	AD7606_CHANNEL(7),
};

static const struct iio_chan_spec ad7616_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(16),
	AD7606_CHANNEL(0),
	AD7606_CHANNEL(1),
	AD7606_CHANNEL(2),
	AD7606_CHANNEL(3),
	AD7606_CHANNEL(4),
	AD7606_CHANNEL(5),
	AD7606_CHANNEL(6),
	AD7606_CHANNEL(7),
	AD7606_CHANNEL(8),
	AD7606_CHANNEL(9),
	AD7606_CHANNEL(10),
	AD7606_CHANNEL(11),
	AD7606_CHANNEL(12),
	AD7606_CHANNEL(13),
	AD7606_CHANNEL(14),
	AD7606_CHANNEL(15),
};

static const struct ad7606_chip_info ad7606_chip_info_tbl[] = {
	/* More devices added in future */
	[ID_AD7605_4] = {
		.channels = ad7605_channels,
		.num_channels = 5,
	},
	[ID_AD7606_8] = {
		.channels = ad7606_channels,
		.num_channels = 9,
		.oversampling_avail = ad7606_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	},
	[ID_AD7606_6] = {
		.channels = ad7606_channels,
		.num_channels = 7,
		.oversampling_avail = ad7606_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	},
	[ID_AD7606_4] = {
		.channels = ad7606_channels,
		.num_channels = 5,
		.oversampling_avail = ad7606_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	},
	[ID_AD7606B] = {
		.channels = ad7606_channels,
		.num_channels = 9,
		.oversampling_avail = ad7606_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7606_oversampling_avail),
	},
	[ID_AD7616] = {
		.channels = ad7616_channels,
		.num_channels = 17,
		.oversampling_avail = ad7616_oversampling_avail,
		.oversampling_num = ARRAY_SIZE(ad7616_oversampling_avail),
		.os_req_reset = true,
		.init_delay_ms = 15,
	},
};



static int ad7606_request_gpios(struct ad7606_state *st)
{
	struct device *dev = st->dev;

	st->gpio_convst = devm_gpiod_get(dev, "adi,conversion-start",
					 GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_convst))
		return PTR_ERR(st->gpio_convst);

	st->gpio_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	st->gpio_range = devm_gpiod_get_optional(dev, "adi,range",
						 GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_range))
		return PTR_ERR(st->gpio_range);

	st->gpio_standby = devm_gpiod_get_optional(dev, "standby",
						   GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_standby))
		return PTR_ERR(st->gpio_standby);

	st->gpio_frstdata = devm_gpiod_get_optional(dev, "adi,first-data",
						    GPIOD_IN);
	if (IS_ERR(st->gpio_frstdata))
		return PTR_ERR(st->gpio_frstdata);

	st->gpio_cs = devm_gpiod_get_optional(dev, "cs",
			GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_cs))
		return PTR_ERR(st->gpio_cs);

	st->gpio_rd = devm_gpiod_get_optional(dev, "rd",
			GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_rd))
		return PTR_ERR(st->gpio_rd);



	if (!st->chip_info->oversampling_num)
		return 0;

	st->gpio_os = devm_gpiod_get_array_optional(dev,
						    "adi,oversampling-ratio",
						    GPIOD_OUT_LOW);
	return PTR_ERR_OR_ZERO(st->gpio_os);
}

/*
 * The BUSY signal indicates when conversions are in progress, so when a rising
 * edge of CONVST is applied, BUSY goes logic high and transitions low at the
 * end of the entire conversion process. The falling edge of the BUSY signal
 * triggers this interrupt.
 */
static irqreturn_t ad7606_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ad7606_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev)) {
		gpiod_set_value(st->gpio_convst, 0);
		iio_trigger_poll_nested(st->trig);
	} else {
		complete(&st->completion);
	}

	return IRQ_HANDLED;
};

static int ad7606_validate_trigger(struct iio_dev *indio_dev,
				   struct iio_trigger *trig)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static int ad7606_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	gpiod_set_value(st->gpio_convst, 1);

	return 0;
}

static int ad7606_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	gpiod_set_value(st->gpio_convst, 0);

	return 0;
}

static int ad7606_read_samples(struct ad7606_state *st)
{
	unsigned int num = st->chip_info->num_channels - 1;
	u16 *data = st->data;

	// TODO : implement channel subset read of channels 0 to n -1;
	// with n <= num_channels
	// effectively disabling the higher index channels, but potentially increasing effective sample rate

	return st->bops->read_block(st->dev, num, data);
}

static int ad7606_scan_direct(struct iio_dev *indio_dev, unsigned int ch)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	gpiod_set_value(st->gpio_convst, 1);
	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(1000));
	if (!ret) {
		ret = -ETIMEDOUT;
		goto error_ret;
	}

	ret = ad7606_read_samples(st);
	if (ret == 0)
		ret = st->data[ch];

error_ret:
	gpiod_set_value(st->gpio_convst, 0);

	return ret;
}

static int ad7606_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan,
	int *val,
	int *val2,
	long m)
{
int ret, ch = 0;
struct ad7606_state *st = iio_priv(indio_dev);

switch (m) {
case IIO_CHAN_INFO_RAW:
iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
 ret = ad7606_scan_direct(indio_dev, chan->address);
 if (ret < 0)
	 return ret;
 *val = (short) ret;
 return IIO_VAL_INT;
}
unreachable();
case IIO_CHAN_INFO_SCALE:
if (st->sw_mode_en)
 ch = chan->address;
*val = 0;
*val2 = st->scale_avail[st->range[ch]];
return IIO_VAL_INT_PLUS_MICRO;
case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
*val = st->oversampling;
return IIO_VAL_INT;
}
return -EINVAL;
}

static int ad7606_write_scale_hw(struct iio_dev *indio_dev, int ch, int val)
{
struct ad7606_state *st = iio_priv(indio_dev);

gpiod_set_value(st->gpio_range, val);

return 0;
}

static int ad7606_write_os_hw(struct iio_dev *indio_dev, int val)
{
struct ad7606_state *st = iio_priv(indio_dev);
DECLARE_BITMAP(values, 3);

values[0] = val & GENMASK(2, 0);

gpiod_set_array_value(st->gpio_os->ndescs, st->gpio_os->desc,
	 st->gpio_os->info, values);

/* AD7616 requires a reset to update value */
if (st->chip_info->os_req_reset)
ad7606_reset(st);

return 0;
}


static int ad7606_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan,
	int val,
	int val2,
	long mask)
{
struct ad7606_state *st = iio_priv(indio_dev);
int i, ret, ch = 0;

guard(mutex)(&st->lock);

switch (mask) {
case IIO_CHAN_INFO_SCALE:
i = find_closest(val2, st->scale_avail, st->num_scales);
if (st->sw_mode_en)
ch = chan->address;
ret = st->write_scale(indio_dev, ch, i);
if (ret < 0)
return ret;
st->range[ch] = i;

return 0;
case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
if (val2)
return -EINVAL;
i = find_closest(val, st->oversampling_avail,
	 st->num_os_ratios);
ret = st->write_os(indio_dev, i);
if (ret < 0)
return ret;

return 0;
default:
return -EINVAL;
}
}


static int ad7606_reg_access(struct iio_dev *indio_dev,
	unsigned int reg,
	unsigned int writeval,
	unsigned int *readval)
{
struct ad7606_state *st = iio_priv(indio_dev);
int ret;

guard(mutex)(&st->lock);

if (readval) {
ret = st->bops->reg_read(st, reg);
if (ret < 0)
return ret;
*readval = ret;
return 0;
} else {
return st->bops->reg_write(st, reg, writeval);
}
}


static const struct iio_buffer_setup_ops ad7606_buffer_ops = {
	.postenable = &ad7606_buffer_postenable,
	.predisable = &ad7606_buffer_predisable,
};

static const struct iio_info ad7606_info_no_os_or_range = {
	.read_raw = &ad7606_read_raw,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_info ad7606_info_os_and_range = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_os_and_range,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_info ad7606_info_os_range_and_debug = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.debugfs_reg_access = &ad7606_reg_access,
	.attrs = &ad7606_attribute_group_os_and_range,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_info ad7606_info_os = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_os,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_info ad7606_info_range = {
	.read_raw = &ad7606_read_raw,
	.write_raw = &ad7606_write_raw,
	.attrs = &ad7606_attribute_group_range,
	.validate_trigger = &ad7606_validate_trigger,
};

static const struct iio_trigger_ops ad7606_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};




static int ad7606_bus_reg_read(struct iio_backend *back, u16 reg, u16 *val)
{
	struct ad7606_state *st = iio_backend_get_priv(back);
	int ret;

	guard(mutex)(&st->lock);

	ret = st->bops->reg_read(st, reg);
	if (ret < 0)
	return ret;
	*val = ret;
	return 0;
}

static int ad7606_bus_reg_write(struct iio_backend *back, u16 reg, u16 val)
{
	struct ad7606_state *st = iio_backend_get_priv(back);
	guard(mutex)(&st->lock);
	return st->bops->reg_write(st, reg, val);
}


/*
static struct ad7606_platform_data ad7606_pdata = {
	.bus_reg_read = ad7606_bus_reg_read,
	.bus_reg_write = ad7606_bus_reg_write,
};
*/

/*
static struct resource ad7606_resources[] = {
    [0] = {
        .start  = 0x3F200035,           // SDP: AMS1 / CS_B 
        .end    = 0x3F200036,
        .flags  = IORESOURCE_MEM,
    },
    [1] = { // general IRQ
        .start  = 4,      // SDP: GPIO4 
        .end    = 4,
        .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
    },
};
 
static struct platform_device ad7606_device = {
    .name       = "ad7606-8",
    .dev = {
     //   .platform_data = &ad7606_pdata,
    },
    .num_resources  = ARRAY_SIZE(ad7606_resources),
    .resource   = ad7606_resources,
};

static struct platform_device *board_devices[] __initdata = {
    &ad7606_device,
};
*/


struct platform_device *_pdev;


int ad7606_reset(struct ad7606_state *st)
{
	if (st->gpio_reset) {
		gpiod_set_value(st->gpio_reset, 1);
		ndelay(100); /* t_reset >= 100ns */
		gpiod_set_value(st->gpio_reset, 0);
		return 0;
	}

	return -ENODEV;
}
EXPORT_SYMBOL_NS_GPL(ad7606_reset, IIO_AD7606);



static irqreturn_t ad7606_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&st->lock);

	ret = ad7606_read_samples(st);
	if (ret)
		goto error_ret;

	iio_push_to_buffers_with_timestamp(indio_dev, st->data,
					   iio_get_time_ns(indio_dev));
error_ret:
	iio_trigger_notify_done(indio_dev->trig);
	/* The rising edge of the CONVST signal starts a new conversion. */
	gpiod_set_value(st->gpio_convst, 1);

	return IRQ_HANDLED;
}



static int ad7606_par16_read_block(struct device *dev,
				   int count, void *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);


	/*
	 * On the parallel interface, the frstdata signal is set to high while
	 * and after reading the sample of the first channel and low for all
	 * other channels.  This can be used to check that the incoming data is
	 * correctly aligned.  During normal operation the data should never
	 * become unaligned, but some glitch or electrostatic discharge might
	 * cause an extra read or clock cycle.  Monitoring the frstdata signal
	 * allows to recover from such failure situations.
	 */


	int num = count; // total number of channels to read.
	u16 *_buf = buf;
	u32 val;
	u16 delay_ns = 200;



	/*
	* CS/RD strobe, assuming single AD7606 on the bus and 8 channel simultaneous sampling, not in 2 groups of 4 channels
	* TODO : check that count is less or equal than the number of channels supported by the device (id->driver_data matching the dt device)
	*
	*/

	gpiod_set_value(st->gpio_cs,0); // chip select set to low. 
	    // TODO : check that line propery is default (active high) in gpio setup
		// add proper timing constraints. test : sleep based and delay functions
		// https://stackoverflow.com/questions/15994603/how-to-sleep-in-the-linux-kernel
		// TODO : check the timing behaviour of that call, or use MMIO to set value directly
	
	ndelay(delay_ns);
	gpiod_set_value(st->gpio_rd,0); // rd set to low, read first channel
	ndelay(delay_ns);

	bool first_channel = true;

	while(num>0)
	{
		if ((bool) gpiod_get_value(st->gpio_frstdata) == first_channel)
		{
			first_channel = false; 

			//insb((unsigned long)st->base_address, _buf, 2);

			val = ioread32(st->base_address);
			*_buf = (u16) ((val >> 8) & 0xFFFF); // extract 16 bits
			_buf++;
			num--;
		}
		else
		{
			gpiod_set_value(st->gpio_cs,1); // putting back cs to active high due to IO error
			gpiod_set_value(st->gpio_rd,1); // putting back rd to active high due to IO error
 
			// TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);		
			ad7606_reset(st);
			// IO error, the IC signals first channel conversion although it's not, or doesn't signal first channel
			// conversion although it should be.
			return -EIO;

		}

		gpiod_set_value(st->gpio_rd,1); // rd strobe, read next channel
		ndelay(delay_ns);
		gpiod_set_value(st->gpio_rd,0);
		ndelay(delay_ns);
		

	}
	gpiod_set_value(st->gpio_cs,1); // data read end, putting back cs to active high
	gpiod_set_value(st->gpio_rd,1); // data read end, putting back rd to active high

	// TODO : check that line propery is default (active high) in gpio setup
	ndelay(delay_ns);
	return 0;

/*
	if (st->gpio_frstdata) {
		insw((unsigned long)st->base_address, _buf, 1);
		if (!gpiod_get_value(st->gpio_frstdata)) {
			ad7606_reset(st);
			return -EIO;
		}
		_buf++;
		num--;
	}
	insw((unsigned long)st->base_address, _buf, num);
	return 0;
*/

}

static const struct ad7606_bus_ops ad7606_par16_bops = {
	.read_block = ad7606_par16_read_block,
};

static int ad7606_par8_read_block(struct device *dev,
				  int count, void *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);
	/*
	 * On the parallel interface, the frstdata signal is set to high while
	 * and after reading the sample of the first channel and low for all
	 * other channels.  This can be used to check that the incoming data is
	 * correctly aligned.  During normal operation the data should never
	 * become unaligned, but some glitch or electrostatic discharge might
	 * cause an extra read or clock cycle.  Monitoring the frstdata signal
	 * allows to recover from such failure situations.
	 */
	int num = count; // total number of channels to read.
	u16 *_buf = buf;
	u32 val;
	u8 lsb, msb;
	u16 delay_ns = 200;



	/*
	 * CS/RD strobe, assuming single AD7606 on the bus and 8 channel simultaneous sampling, not in 2 groups of 4 channels
	 * TODO : check that count is less or equal than the number of channels supported by the device (id->driver_data matching the dt device)
	 *
	 */

	gpiod_set_value(st->gpio_cs,0); // TODO : check that line propery is default (active high) in gpio setup
		// add proper timing constraints. test : sleep based and delay functions
		// https://stackoverflow.com/questions/15994603/how-to-sleep-in-the-linux-kernel
	
	ndelay(delay_ns);
	
	// strobe cycle 0.4us, total read time for all channels = 0.4us * 2 * 8 reads, 12.8us, which gives max sampling rate of 156.250 ksps per channel
	// not accounting preamble delays (convst signal )
	// management, etc. so probably 100 ksps being conservative.
	// higher sampling rates should implement shorter RPI to ADC ribbons, and shielding or twisted pair, and/or a guard grounded conductor 
	// on one side between the ADC and each pin of the parallel interface.

	// coded for mode with 8 bit lsb read, first the 8 bit msb read next.
	// that means DB14/HBEN pin of the ADC7606 SHALL be tied to GND
	// TODO : code the opposite mode, or hardcode and don't to make the code a little faster, with less configuration
	// overhead.

	// TODO : performance testing writing directly to mmio set value register for the CS/RD strobe pin
	// instead of gpiod_set_value()

	bool first_channel = true;

	while(num>0)
	{
		if ((bool) gpiod_get_value(st->gpio_frstdata) == first_channel)
		{
			first_channel = false; 

			//insb((unsigned long)st->base_address, _buf, 2);

			val = ioread32(st->base_address);
			lsb = (val >> 8) & 0xFF; // extract 8 bits (lsb)
			gpiod_set_value(st->gpio_rd,1); // TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);
			gpiod_set_value(st->gpio_rd,0); // TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);

			val = ioread32(st->base_address);
			msb = (val >> 8) & 0xFF; // extract 8 bits (msb)
			gpiod_set_value(st->gpio_rd,1); // TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);
			gpiod_set_value(st->gpio_rd,0); // TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);
			// at this point frstdata should be low (after falling edge of CS/RD above, per datasheet)		

			*_buf = ((u16) msb << 8) | lsb; // combine into 16 bit channel sample value
			_buf++;
			num--;
		}
		else
		{
			gpiod_set_value(st->gpio_cs,1); // TODO : check that line propery is default (active high) in gpio setup
			gpiod_set_value(st->gpio_rd,1); // TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);		
			ad7606_reset(st);
			// IO error, the IC signals first channel conversion although it's not, or doesn't signal first channel
			// conversion although it should be.
			return -EIO;

		}

	}
	gpiod_set_value(st->gpio_cs,1); // TODO : check that line propery is default (active high) in gpio setup
	gpiod_set_value(st->gpio_rd,1); // TODO : check that line propery is default (active high) in gpio setup			
	ndelay(delay_ns);
	
}

static const struct ad7606_bus_ops ad7606_par8_bops = {
	.read_block = ad7606_par8_read_block,
};


/*
static inline int ad7606_par_probe(struct device *dev, int irq, void __iomem *base_address,
	const char *name, unsigned int id,
	const struct ad7606_bus_ops *bops)
{
struct ad7606_state *st;
int ret;
struct iio_dev *indio_dev;

dev_dbg(dev,"ad7606:entered ad7606_probe()\n");

indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
if (!indio_dev)
   return -ENOMEM;

dev_dbg(dev,"ad7606:devm_iio_device_allloc() ok\n");
st = iio_priv(indio_dev);
dev_set_drvdata(dev, indio_dev);

st->dev = dev;
mutex_init(&st->lock);
st->bops = bops;
st->base_address = base_address;
// tied to logic low, analog input range is +/- 5V 
st->range[0] = 0;
st->oversampling = 1;
st->scale_avail = ad7606_scale_avail;
st->num_scales = ARRAY_SIZE(ad7606_scale_avail);

ret = devm_regulator_get_enable(dev, "avcc");
if (ret)
   return dev_err_probe(dev, ret,
				"Failed to enable specified AVcc supply\n");

dev_dbg(dev,"ad7606:devm_regulator_get_enable() ok\n");
st->chip_info = &ad7606_chip_info_tbl[id];

if (st->chip_info->oversampling_num) {
   st->oversampling_avail = st->chip_info->oversampling_avail;
   st->num_os_ratios = st->chip_info->oversampling_num;
}

ret = ad7606_request_gpios(st);
if (ret)
   return ret;


dev_dbg(dev,"ad7606:ad7606_request_gpios() ok\n");

if (st->gpio_os) {
   if (st->gpio_range)
	   indio_dev->info = &ad7606_info_os_and_range;
   else
	   indio_dev->info = &ad7606_info_os;
} else {
   if (st->gpio_range)
	   indio_dev->info = &ad7606_info_range;
   else
	   indio_dev->info = &ad7606_info_no_os_or_range;
}
indio_dev->modes = INDIO_DIRECT_MODE;
indio_dev->name = name;
indio_dev->channels = st->chip_info->channels;
indio_dev->num_channels = st->chip_info->num_channels;

init_completion(&st->completion);

ret = ad7606_reset(st);
if (ret)
   dev_warn(st->dev, "failed to RESET: no RESET GPIO specified\n");


dev_dbg(dev,"ad7606:ad760_reset() ok\n");

// AD7616 requires al least 15ms to reconfigure after a reset
if (st->chip_info->init_delay_ms) {
   if (msleep_interruptible(st->chip_info->init_delay_ms))
	   return -ERESTARTSYS;
}

st->write_scale = ad7606_write_scale_hw;
st->write_os = ad7606_write_os_hw;

if (st->bops->sw_mode_config)
   st->sw_mode_en = device_property_present(st->dev,
						"adi,sw-mode");

if (st->sw_mode_en) {
   // Scale of 0.076293 is only available in sw mode
   st->scale_avail = ad7616_sw_scale_avail;
   st->num_scales = ARRAY_SIZE(ad7616_sw_scale_avail);

   // After reset, in software mode, ±10 V is set by default
   memset32(st->range, 2, ARRAY_SIZE(st->range));
   indio_dev->info = &ad7606_info_os_range_and_debug;

   ret = st->bops->sw_mode_config(indio_dev);
   if (ret < 0)
	   return ret;
}

st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
				 indio_dev->name,
				 iio_device_id(indio_dev));
if (!st->trig)
   return -ENOMEM;

dev_dbg(dev,"ad7606:devm_iio_trigger_allloc() ok\n");

st->trig->ops = &ad7606_trigger_ops;
iio_trigger_set_drvdata(st->trig, indio_dev);
ret = devm_iio_trigger_register(dev, st->trig);
if (ret)
   return ret;

indio_dev->trig = iio_trigger_get(st->trig);

ret = devm_request_threaded_irq(dev, irq,
			   NULL,
			   &ad7606_interrupt,
			   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			   name, indio_dev);
if (ret)
   return ret;

dev_dbg(dev,"ad7606:devm_request_threaded_irq() ok\n");

ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					 &iio_pollfunc_store_time,
					 &ad7606_trigger_handler,
					 &ad7606_buffer_ops);
if (ret)
   return ret;


dev_dbg(dev,"ad7606:devm_iio_triggered_buffer_setup() ok\n");

return devm_iio_device_register(dev, indio_dev);
}
*/

#define GPIOGET   0x34
#define GPIO_GET() readl((addr + GPIOGET)) // get the value of the pin.


int ad7606_probe(struct platform_device *pdev)
{
	

	_pdev = pdev;
	dev_dbg(&pdev->dev,"ad7606:entered ad7606_par_probe\n");
	const struct platform_device_id *id = platform_get_device_id(pdev);

	if (IS_ERR(id))
	{
		dev_dbg(&pdev->dev,"platform_get_device_id() failed !\n");
		return PTR_ERR(id);
	}

	struct resource *res;
	void __iomem *addr;
	resource_size_t remap_size;

	int ret;
	int irq;
	
	/*
	int gpio;

	gpio = ad7606_resources[1].start;

	dev_dbg(&pdev->dev,"irq from ad7606_resources[1].start ok = %i\n",gpio);

	const char gpio_busy_label[] = "ad7606_busy";

	ret = gpio_request(gpio, gpio_busy_label);
	if (IS_ERR_VALUE(ret))
	{
		dev_dbg(&pdev->dev, "gpio_request() failed = %i\n",ret);
		return ret;
	}

	dev_dbg(&pdev->dev, "gpio_request() ok = %i\n",ret);
	
	ret = gpio_direction_input(gpio);

	if (IS_ERR_VALUE(ret))
	{
		dev_dbg(&pdev->dev,"gpio_direction_input() failed = %i\n",ret);
		return ret;
	}

	dev_dbg(&pdev->dev,"gpio_direction_input() ok = %i\n",ret);

	irq = gpio_to_irq(gpio);

	if (IS_ERR_VALUE(irq))
	{
		dev_dbg(&pdev->dev, "gpio_to_irq() failed = %i\n",irq);
		return irq;
	}
	*/

	irq = platform_get_irq(pdev, 0);
	if (IS_ERR_VALUE(irq))
	{
		dev_dbg(&pdev->dev,"platform_get_irq() failed, err = %i\n",irq);
		return irq;
	}

	dev_dbg(&pdev->dev,"platform_get_irq() ok, irq = %i\n",irq);

	//addr = devm_platform_get_and_ioremap_resource(pdev, 0, &res);

	unsigned long phy_addr;
	struct resource *res2;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phy_addr = res->start;

	dev_dbg(&pdev->dev,"platform_get_resource() ok, start_addr = %lu\n",phy_addr);
	dev_dbg(&pdev->dev,"will request mem region(), for device name = %s\n",dev_name(&pdev->dev));
	
    res2 = request_mem_region(phy_addr, 4, dev_name(&pdev->dev));
    // check for errors

	if (IS_ERR(res2))
		return PTR_ERR(res2);

	dev_dbg(&pdev->dev,"request mem region ok()\n");

        addr = ioremap(phy_addr, 4);

	if (IS_ERR(addr))
		return PTR_ERR(addr);

	//unsigned long remapped_addr = *((unsigned long *) addr);
	dev_dbg(&pdev->dev,"ioremap() ok, ptr is = %p\n",addr);
	//dev_dbg(&pdev->dev,"ioremap() ok, addr is = %lu\n",remapped_addr);

	/*
	if(!remapped_addr)
	{
		dev_dbg(&pdev->dev,"invalid remapped_addr!\n");
		return -1;	
	}
	*/

	remap_size = resource_size(res);

	dev_dbg(&pdev->dev,"resource_size() ok, remap_size = %i\n",(int) remap_size);

	u32 testval = GPIO_GET();

	dev_dbg(&pdev->dev,"readl() test val = %u\n",testval);

	dev_dbg(&pdev->dev,"ad7606:entered ad7606_probe()\n");

	struct ad7606_state *st;
	struct iio_dev *indio_dev;
	
	dev_dbg(&pdev->dev,"ad7606:declared structs() ok\n");

	
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	
	dev_dbg(&pdev->dev,"ad7606:devm_iio_device_allloc() ok\n");
	st = iio_priv(indio_dev);
	dev_set_drvdata(&pdev->dev, indio_dev);
	
	st->dev = &pdev->dev;
	mutex_init(&st->lock);
	st->bops = &ad7606_par16_bops;
	st->base_address = addr;
	/* tied to logic low, analog input range is +/- 5V */
	st->range[0] = 0;
	st->oversampling = 1;
	st->scale_avail = ad7606_scale_avail;
	st->num_scales = ARRAY_SIZE(ad7606_scale_avail);
	
	ret = devm_regulator_get_enable(&pdev->dev, "avcc");
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
					"Failed to enable specified AVcc supply\n");
	
	dev_dbg(&pdev->dev,"ad7606:devm_regulator_get_enable() ok\n");
	dev_dbg(&pdev->dev,"chip_id is hardcoded = ID_AD7606_8 \n");
	
	//BUG : the id->driver_data pointer has an issue, seems unable to fetch from the dts, or a struct/pointer nesting issue.
	// WORKAROUND : hardcode id driver_data to 1 = ID_AD7606_8
	//st->chip_info = &ad7606_chip_info_tbl[id->driver_data];
	st->chip_info = &ad7606_chip_info_tbl[ID_AD7606_8];


	dev_dbg(&pdev->dev,"ad7606:state chip info set() ok\n");
	
	if (st->chip_info->oversampling_num) {
		st->oversampling_avail = st->chip_info->oversampling_avail;
		dev_dbg(&pdev->dev,"ad7606:state oversampling_avail set() ok\n");
		st->num_os_ratios = st->chip_info->oversampling_num;
		dev_dbg(&pdev->dev,"ad7606:state num_os_ratios set() ok\n");

	}
	
	ret = ad7606_request_gpios(st);
	if (ret)
		return ret;
	
	dev_dbg(&pdev->dev,"ad7606:ad7606_request_gpios() ok\n");
	
	if (st->gpio_os) {
		if (st->gpio_range)
			indio_dev->info = &ad7606_info_os_and_range;
		else
			indio_dev->info = &ad7606_info_os;
	} else {
		if (st->gpio_range)
			indio_dev->info = &ad7606_info_range;
		else
			indio_dev->info = &ad7606_info_no_os_or_range;
	}
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = id->name;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	
	init_completion(&st->completion);
	
	ret = ad7606_reset(st);
	if (ret)
		dev_warn(st->dev, "failed to RESET: no RESET GPIO specified\n");
	
	
	dev_dbg(&pdev->dev,"ad7606:ad760_reset() ok\n");
	
	/* AD7616 requires al least 15ms to reconfigure after a reset */
	if (st->chip_info->init_delay_ms) {
		if (msleep_interruptible(st->chip_info->init_delay_ms))
			return -ERESTARTSYS;
	}
	
	st->write_scale = ad7606_write_scale_hw;
	st->write_os = ad7606_write_os_hw;
	
	if (st->bops->sw_mode_config)
		st->sw_mode_en = device_property_present(st->dev,
							"adi,sw-mode");
	
	if (st->sw_mode_en) {
		/* Scale of 0.076293 is only available in sw mode */
		st->scale_avail = ad7616_sw_scale_avail;
		st->num_scales = ARRAY_SIZE(ad7616_sw_scale_avail);
	
		/* After reset, in software mode, ±10 V is set by default */
		memset32(st->range, 2, ARRAY_SIZE(st->range));
		indio_dev->info = &ad7606_info_os_range_and_debug;
	
		ret = st->bops->sw_mode_config(indio_dev);
		if (ret < 0)
			return ret;
	}
	
	st->trig = devm_iio_trigger_alloc(&pdev->dev, "%s-dev%d",
						indio_dev->name,
						iio_device_id(indio_dev));
	if (!st->trig)
		return -ENOMEM;
	
	dev_dbg(&pdev->dev,"ad7606:devm_iio_trigger_allloc() ok\n");
	
	st->trig->ops = &ad7606_trigger_ops;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(&pdev->dev, st->trig);
	if (ret)
		return ret;
	
	indio_dev->trig = iio_trigger_get(st->trig);
	
	ret = devm_request_threaded_irq(&pdev->dev, irq,
					NULL,
					&ad7606_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					id->name, indio_dev);
	if (ret)
		return ret;
	
	dev_dbg(&pdev->dev,"ad7606:devm_request_threaded_irq() ok\n");
	
	ret = devm_iio_triggered_buffer_setup(&pdev->dev, indio_dev,
							&iio_pollfunc_store_time,
							&ad7606_trigger_handler,
							&ad7606_buffer_ops);
	if (ret)
		return ret;
	
	
	dev_dbg(&pdev->dev,"ad7606:devm_iio_triggered_buffer_setup() ok\n");
	
	return devm_iio_device_register(&pdev->dev, indio_dev);
			
	
	dev_dbg(&pdev->dev,"inline call ad7606_par_probe() ok \n");
	return ret;

}

EXPORT_SYMBOL_NS_GPL(ad7606_probe, IIO_AD7606);

static const struct platform_device_id ad7606_driver_ids[] = {
	{ .name	= "ad7605-4", .driver_data = ID_AD7605_4, },
	{ .name	= "ad7606-4", .driver_data = ID_AD7606_4, },
	{ .name	= "ad7606-6", .driver_data = ID_AD7606_6, },
	{ .name	= "ad7606-8", .driver_data = ID_AD7606_8, },
	{ }
};
MODULE_DEVICE_TABLE(platform, ad7606_driver_ids);

static const struct of_device_id ad7606_of_match[] = {
	{ .compatible = "adi,ad7605-4" },
	{ .compatible = "adi,ad7606-4" },
	{ .compatible = "adi,ad7606-6" },
	{ .compatible = "adi,ad7606-8" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad7606_of_match);

static struct platform_driver ad7606_driver = {
	.probe = ad7606_probe,
	.id_table = ad7606_driver_ids,
	.driver = {
		.name = "ad7606",
		.pm = AD7606_PM_OPS,
		.of_match_table = ad7606_of_match,
	},
};

static int __init board_init(void)
{
    printk(KERN_INFO "ad7606_par module init.");
	
	//platform_add_devices(board_devices, ARRAY_SIZE(board_devices));
	printk(KERN_INFO "platform_add_devices() called.");
	platform_driver_register(&ad7606_driver);
	printk(KERN_INFO "platform_driver_register() called.");
	

    return 0;
}

static void board_unload(void)
{
    printk(KERN_INFO "ad7606_par module unload.");
	
	platform_device_unregister(_pdev);
	printk(KERN_INFO "platform_device_unregister() called.");
	platform_driver_unregister(&ad7606_driver);
	printk(KERN_INFO "platform_driver_unregister() called.");
	
}

//arch_initcall(board_init);
module_init(board_init);
module_exit(board_unload);

//module_platform_driver(ad7606_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606 ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_AD7606);
