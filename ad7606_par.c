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
#define TIMINGDELAY 20000

#include <linux/types.h>
#include <linux/property.h>
#include <linux/sizes.h>
#include <linux/bitmap.h>


#include <linux/io.h>
#include <linux/ioport.h>


#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of_device.h>

#include <linux/err.h>

#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>

#include <linux/slab.h>
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
#include <linux/iio/buffer_impl.h>




#include "ad7606.h"
#include "ad7606_bus_iface.h"

#define GPIOSET(x,y) gpiod_set_raw_value(x, y)
#define GPIOSETARR(w,x,y,z) gpiod_set_raw_array_value(w, x, y, z)


static long cs_rd_values_zero[1];
static long cs_rd_values_one[1];


// x = pin_desc or desc array
// y = value
// z = number of pins in the array

// gpiod_set_raw_value is preferred as all pins are active_high besides standby, and standby is not used.
// would probably give a handful of cpu cycles gain per call
// ACTIVE_LOW / ACTIVE_HIGH specification would remain in device tree solely for informational purposes.

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
	dev_dbg(dev,"ad7606:entered ad7606_request_gpios()\n");
	

	st->gpio_convst = devm_gpiod_get(dev, "adi,conversion-start",
					 GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_convst))
		return PTR_ERR(st->gpio_convst);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_convst set\n");
	

	st->gpio_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_reset set\n");


	st->gpio_range = devm_gpiod_get_optional(dev, "adi,range",
						 GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_range))
		return PTR_ERR(st->gpio_range);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_range set\n");


	st->gpio_standby = devm_gpiod_get_optional(dev, "standby",
						   GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_standby))
		return PTR_ERR(st->gpio_standby);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_standby set\n");

	st->gpio_frstdata = devm_gpiod_get_optional(dev, "adi,first-data",
						    GPIOD_IN);
	if (IS_ERR(st->gpio_frstdata))
		return PTR_ERR(st->gpio_frstdata);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_frstdata set\n");

/*
	st->gpio_cs = devm_gpiod_get_optional(dev, "cs",
			GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_cs))
		return PTR_ERR(st->gpio_cs);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_cs set\n");


	st->gpio_rd = devm_gpiod_get_optional(dev, "rd",
			GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_rd))
		return PTR_ERR(st->gpio_rd);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_rd set\n");
*/

	//st->gpio_cs_rd[0] = st->gpio_cs;
	//st->gpio_cs_rd[1] = st->gpio_rd;
	

	st->gpio_cs_rd = devm_gpiod_get_array(dev,
		"cs-rd",
		GPIOD_OUT_LOW);
	if(IS_ERR(st->gpio_cs_rd))
		return PTR_ERR(st->gpio_cs_rd);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_cs_rd set\n");

	st->gpio_cs = st->gpio_cs_rd->desc[0];
	st->gpio_rd = st->gpio_cs_rd->desc[1];
	dev_dbg(dev,"ad7606:ad7606_request_gpios():individual gpio_cs and gpio_rd set\n");
	
	//TODO: put bitmap in state
	bitmap_zero(cs_rd_values_zero,2);
	bitmap_zero(cs_rd_values_one,2);
	cs_rd_values_one[0] = GENMASK(1, 0);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():cs_rd bitmaps set\n");


	st->gpio_parallel_data = devm_gpiod_get_array(dev,
		"adi,parallel-data",
		GPIOD_IN);
	if(IS_ERR(st->gpio_parallel_data))
		return PTR_ERR(st->gpio_parallel_data);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_parallel_data set\n");


	if (!st->chip_info->oversampling_num)
		return 0;

	st->gpio_os = devm_gpiod_get_array_optional(dev,
						    "adi,oversampling-ratio",
						    GPIOD_OUT_LOW);
	dev_dbg(dev,"ad7606:ad7606_request_gpios():gpio_os set\n");

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
	pr_debug("ad7606:enter ad7606_interrupt\n");
	
	struct iio_dev *indio_dev = dev_id;
	pr_debug("ad7606:get indio_dev\n");
	
	struct ad7606_state *st = iio_priv(indio_dev);
	pr_debug("ad7606:get state\n");


	/*
	TEST : this was internal trigger old code
	if (iio_buffer_enabled(indio_dev)) {
		iio_trigger_poll_nested(st->trig);
		pr_debug("ad7606:trigger_poll_nested\n");

	} else {
		complete(&st->completion);
	}
	*/

	// TEST : external trigger code, we signal the trigger handler that conversion is done
	complete(&st->completion);
	// END TEST

	pr_debug("ad7606:return IRQ_HANDLED\n");

	return IRQ_HANDLED;
};

static int ad7606_validate_trigger(struct iio_dev *indio_dev,
				   struct iio_trigger *trig)
{
	struct ad7606_state *st = iio_priv(indio_dev);

	//if (st->trig != trig)
	//	return -EINVAL;

	//return 0;
	int ret;
	pr_debug("ad7606:validating trigger:%s\n",trig->name);
	ret = strcmp(trig->name,"hrtimer0");
	pr_debug("ad7606:validating trigger:%u\n",ret);
	if (ret == 0)
	{
		pr_debug("ad7606:setting trigger to state:%u\n",ret);
		st->trig = trig;
	}

	return !(ret == 0);// DEBUGGING external trigger setup
	
}

static int ad7606_buffer_postenable(struct iio_dev *indio_dev)
{
	
	struct ad7606_state *st = iio_priv(indio_dev);
	
	
	// dirty fix, increment module refcount to prevent module unload (such as done by rmmod)
	// while buffer is enabled.
	if(!try_module_get(THIS_MODULE))
	{
		dev_warn(st->dev, "buffer is enabled, but failed to prevent future module unloading while it is enabled\n");
	} 
	
	if(indio_dev->scan_bytes)
	{
		st->data_scan_elements = kmalloc(indio_dev->scan_bytes + 8,GFP_KERNEL); // plus 8 bytes for TIMESTAMP.
		if(!st->data_scan_elements) {return -ENOMEM;}
		st->buffer_enabled = true;
	}
	else
	{
		dev_warn(st->dev, "cannot enable buffer with zero scan elements!\n");
		// TODO : TEST : check that this condition is not already managed upstream by iio buffer management
		return -EINVAL;
	}

	GPIOSET(st->gpio_convst, 1);
	ndelay(TIMINGDELAY);

	return 0;
}

static int ad7606_buffer_predisable(struct iio_dev *indio_dev)
{

	struct ad7606_state *st = iio_priv(indio_dev);
	
	
	kfree(st->data_scan_elements);
	GPIOSET(st->gpio_convst, 0);
	ndelay(TIMINGDELAY);
	st->buffer_enabled = false;
	//dirty fix : decrement module refcount to allow unloading, since the buffer is disabled
	module_put(THIS_MODULE);
	

	return 0;
}

static int ad7606_read_samples(struct ad7606_state *st)
{
	unsigned int num = st->chip_info->num_channels - 1;
	s16 *data = st->data;

	// TODO : implement channel subset read of channels 0 to n -1;
	// with n <= num_channels, if the hardware protocol permits.
	// effectively disabling the higher index channels, but potentially increasing effective sample rate

	return st->bops->read_block(st->dev, num, data);
}

static int ad7606_scan_direct(struct iio_dev *indio_dev, unsigned int ch)
{
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	GPIOSET(st->gpio_convst, 1);
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
	GPIOSET(st->gpio_convst, 0);

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

GPIOSET(st->gpio_range, val);

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


//struct platform_device *_pdev;
u32 __iomem *addr;
unsigned long phy_addr;



int ad7606_reset(struct ad7606_state *st)
{
	dev_warn(st->dev,"ad7606_reset called!");

	if (st->gpio_reset) {
		GPIOSET(st->gpio_reset, 1);
		ndelay(TIMINGDELAY); /* t_reset >= 100ns */
		GPIOSET(st->gpio_reset, 0);
		return 0;
	}

	return -ENODEV;
}
EXPORT_SYMBOL_NS_GPL(ad7606_reset, IIO_AD7606);



static irqreturn_t ad7606_trigger_handler(int irq, void *p)
{
	// TEST : modifying code to process an external trigger such as hrtimer, not the device internally managed trigger.
	// this should be the entry point when a timer expires.
	
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7606_state *st = iio_priv(indio_dev);
	int ret;

	//struct timespec64 ts1;
	//struct timespec64 ts2;
	
	//ktime_get_ts64(&ts1);
	

	//dev_info(st->dev,"enter ad7606_trigger_handler\n");

	guard(mutex)(&st->lock);

	// TEST external trigger : initiate a conversion by strobing convst
	GPIOSET(st->gpio_convst, 0);
	ndelay(TIMINGDELAY);
	GPIOSET(st->gpio_convst, 1);
	// now wait for IRQ to fire, signaling BUSY falling edge, and end of conversion. the irq handler will set completion
	ret = wait_for_completion_timeout(&st->completion,
		msecs_to_jiffies(1000));
	if (!ret) {
		ret = -ETIMEDOUT;
		goto error_ret;
	}
	// end TEST external trigger
	// conversion has ended, read samples

	ret = ad7606_read_samples(st);
	if (ret)
		goto error_ret;

	u8 chan;
	u8 scan_idx = 0;
	for_each_set_bit(chan,indio_dev->active_scan_mask,indio_dev->masklength)
	{
		st->data_scan_elements[scan_idx++] = st->data[chan];
	}

	iio_push_to_buffers_with_timestamp(indio_dev, st->data_scan_elements,
					   iio_get_time_ns(indio_dev));


	//	iio_push_to_buffers_with_timestamp(indio_dev, st->data,
	//					   iio_get_time_ns(indio_dev));
	// TODO : use pointer to store iio_get_time_ns() timestamp closer to the end of conversion process
	
	//ktime_get_ts64(&ts2);

	//dev_info(st->dev,"ad7606_trigger_handler exec start:[%5lld.%06ld]\n",(s64) ts1.tv_sec,ts1.tv_nsec/1000);
	//dev_info(st->dev,"ad7606_trigger_handler exec stop:[%5lld.%06ld]\n",(s64) ts2.tv_sec,ts2.tv_nsec/1000);
	

error_ret:
	iio_trigger_notify_done(indio_dev->trig);
	
	// TEST external trigger
	//// The rising edge of the CONVST signal starts a new conversion.
	// gpiod_set_value(st->gpio_convst, 1);
	// END TEST external trigger

	return IRQ_HANDLED;
}



static int ad7606_par16_read_block(struct device *dev,
				   int count, void *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad7606_state *st = iio_priv(indio_dev);

	dev_dbg(dev,"ad7606:entered ad7606_par16_read_block()\n");
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
	s16 *_buf = buf;
	u32 val;
	u16 delay_ns = TIMINGDELAY;



	/*
	* CS/RD strobe, assuming single AD7606 on the bus and 8 channel simultaneous sampling, not in 2 groups of 4 channels
	* TODO : check that count is less or equal than the number of channels supported by the device (id->driver_data matching the dt device)
	*
	*/

	//GPIOSET(st->gpio_cs,0); // chip select set to low. 
		// add proper timing constraints. test : sleep based and delay functions
		// https://stackoverflow.com/questions/15994603/how-to-sleep-in-the-linux-kernel
		// TODO : check the timing behaviour of that call, or use MMIO to set value directly
	
	//ndelay(delay_ns);
	//GPIOSET(st->gpio_rd,0); // rd set to low, read first channel
	//GPIOSETARR(st->gpio_cs_rd->ndescs,st->gpio_cs_rd->desc,st->gpio_cs_rd->info,cs_rd_values_zero);
	
	//ndelay(delay_ns);

	preempt_disable();
	bool first_channel = true;

	while(num>0)
	{

		//GPIOSET(st->gpio_rd,0);
		GPIOSETARR(st->gpio_cs_rd->ndescs,st->gpio_cs_rd->desc,st->gpio_cs_rd->info,cs_rd_values_zero);
		ndelay(delay_ns);
		

		if ((bool) gpiod_get_value(st->gpio_frstdata) == first_channel)
		{
			first_channel = false; 

			//insb((unsigned long)st->base_address, _buf, 2);

			val = ioread32(st->base_address);
			dev_dbg(dev,"ad7606:channel read. num=%u\n",num);
			dev_dbg(dev,"ad7606:channel read. raw base_address val=%u\n",val);
			

			*_buf = (s16) ((val >> 8) & 0xFFFF); // extract 16 bits
			//*_buf = 0; // extract 16 bits
			
			dev_dbg(dev,"ad7606:channel read. 16 bit shift/mask val=%hd\n",*_buf);

			_buf++;
			num--;
		}
		else
		{

			//if ((bool) gpiod_get_value(st->gpio_frstdata) == first_channel)
			//{
				// DEBUG FRSTDATA INSTABILITY values disagree, retry.
			//	first_channel = false;
			//	continue;
			//}
			dev_warn(dev,"ad7606:channel read. IO error, frstdata level not expected at num=%u\n",num);
			
			//GPIOSET(st->gpio_cs,1); // putting back cs to active high due to IO error
			//GPIOSET(st->gpio_rd,1); // putting back rd to active high due to IO error
			GPIOSETARR(st->gpio_cs_rd->ndescs,st->gpio_cs_rd->desc,st->gpio_cs_rd->info,cs_rd_values_one);
	

			// TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);		
			ad7606_reset(st);
			ndelay(delay_ns);		
			
			// IO error, the IC signals first channel conversion although it's not, or doesn't signal first channel
			// conversion although it should be.
			preempt_enable();
			return -EIO;

		}
		//GPIOSET(st->gpio_rd,1); // rd strobe, read next channel
		GPIOSETARR(st->gpio_cs_rd->ndescs,st->gpio_cs_rd->desc,st->gpio_cs_rd->info,cs_rd_values_one);
		ndelay(delay_ns);	
		

	}
	//GPIOSET(st->gpio_cs,1); // data read end, putting back cs to active high
	//GPIOSET(st->gpio_rd,1); // data read end, putting back rd to active high
	//GPIOSETARR(st->gpio_cs_rd->ndescs,st->gpio_cs_rd->desc,st->gpio_cs_rd->info,cs_rd_values_one);
	
	// TODO : check that line propery is default (active high) in gpio setup
	ndelay(delay_ns);
	dev_dbg(dev,"ad7606:all channels read. exiting callback\n");
	preempt_enable();
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
	s16 *_buf = buf;
	u32 val;
	u8 lsb, msb;
	u16 delay_ns = 200;



	/*
	 * CS/RD strobe, assuming single AD7606 on the bus and 8 channel simultaneous sampling, not in 2 groups of 4 channels
	 * TODO : check that count is less or equal than the number of channels supported by the device (id->driver_data matching the dt device)
	 *
	 */

	GPIOSET(st->gpio_cs,0); // TODO : check that line propery is default (active high) in gpio setup
		// add proper timing constraints. test : sleep based and delay functions
		// https://stackoverflow.com/questions/15994603/how-to-sleep-in-the-linux-kernel
	
	ndelay(delay_ns);


	GPIOSET(st->gpio_rd,0);
	
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
			 
			//insb((unsigned long)st->base_address, _buf, 2);

			val = readl(st->base_address);
			lsb = (val >> 8) & 0xFF; // extract 8 bits (lsb)
			GPIOSET(st->gpio_rd,1); // TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);
			GPIOSET(st->gpio_rd,0); // TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);

			val = readl(st->base_address);
			msb = (val >> 8) & 0xFF; // extract 8 bits (msb)
			GPIOSET(st->gpio_rd,1); // TODO : check that line propery is default (active high) in gpio setup
			ndelay(delay_ns);
			if(num != 1) 
			{
				GPIOSET(st->gpio_rd,0); // TODO : check that line propery is default (active high) in gpio setup
				ndelay(delay_ns);
			}
			// at this point frstdata should be low (after falling edge of CS/RD above, per datasheet)		

			*_buf = ((s16) msb << 8) | lsb; // combine into 16 bit channel sample value
			_buf++;
			num--;
			first_channel = false;
		}
		else
		{
			//GPIOSET(st->gpio_cs,1); // TODO : check that line propery is default (active high) in gpio setup
			//GPIOSET(st->gpio_rd,1); // TODO : check that line propery is default (active high) in gpio setup
			GPIOSETARR(st->gpio_cs_rd->ndescs,st->gpio_cs_rd->desc,st->gpio_cs_rd->info,cs_rd_values_one);
	

			ndelay(delay_ns);		
			
			ad7606_reset(st);
			// IO error, the IC signals first channel conversion although it's not, or doesn't signal first channel
			// conversion although it should be.
			return -EIO;

		}

	}
	GPIOSET(st->gpio_cs,1); // TODO : check that line propery is default (active high) in gpio setup
	//GPIOSET(st->gpio_rd,1); // TODO : check that line propery is default (active high) in gpio setup			
	//GPIOSETARR(st->gpio_cs_rd->ndescs,st->gpio_cs_rd->desc,st->gpio_cs_rd->info,cs_rd_values_one);
	

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
	{ .compatible = "adi,ad7605-4", .data = (void *) &ad7606_driver_ids[0] },
	{ .compatible = "adi,ad7606-4", .data = (void *) &ad7606_driver_ids[1] },
	{ .compatible = "adi,ad7606-6", .data = (void *) &ad7606_driver_ids[2] },
	{ .compatible = "adi,ad7606-8", .data = (void *) &ad7606_driver_ids[3] },
	{ }
};
MODULE_DEVICE_TABLE(of, ad7606_of_match);



static struct platform_driver ad7606_driver = {
	.probe = ad7606_probe,
	.remove = ad7606_remove,
	.id_table = ad7606_driver_ids,
	.driver = {
		.name = "ad7606",
		.pm = AD7606_PM_OPS,
		.of_match_table = ad7606_of_match,
	},
};

void ad7606_remove(struct platform_device *pdev)
{
	//struct iio_dev *indio_dev = dev_get_drvdata(pdev->dev);
	////struct iio_dev *indio_dev = dev_to_iio_dev(pdev->dev);
	//struct ad7606_state *st = iio_priv(indio_dev);

}

int ad7606_probe(struct platform_device *pdev)
{
	

	//_pdev = pdev;
	dev_dbg(&pdev->dev,"ad7606:entered ad7606_par_probe\n");
	//const struct platform_device_id *id = platform_get_device_id(pdev);
	const struct of_device_id *of_id = of_match_device(ad7606_of_match,&pdev->dev);
	const struct platform_device_id *id = of_id ? of_id->data: NULL;
	
	if (IS_ERR(id))
	{
		dev_dbg(&pdev->dev,"of_match_device() failed !\n");
		return PTR_ERR(id);
	}

	struct resource *res;
	resource_size_t remap_size;

	int ret;
	int irq;
	
	irq = platform_get_irq(pdev, 0);
	if (IS_ERR_VALUE(irq))
	{
		dev_dbg(&pdev->dev,"platform_get_irq() failed, err = %i\n",irq);
		return irq;
	}

	dev_dbg(&pdev->dev,"platform_get_irq() ok, irq = %i\n",irq);

	//addr = devm_platform_get_and_ioremap_resource(pdev, 0, &res);

	
	struct resource *res2;
	

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phy_addr = res->start;

	dev_dbg(&pdev->dev,"platform_get_resource() ok, start_addr = %lu\n",phy_addr);
	dev_dbg(&pdev->dev,"will request mem region(), for device name = %s\n",dev_name(&pdev->dev));
	
    res2 = devm_request_mem_region(&pdev->dev, phy_addr, SZ_4K, dev_name(&pdev->dev));
    // check for errors

	if (IS_ERR(res2))
		return PTR_ERR(res2);

	dev_dbg(&pdev->dev,"request mem region ok()\n");

        addr = devm_ioremap(&pdev->dev,phy_addr, SZ_4K);

	if (IS_ERR(addr))
		return PTR_ERR(addr);

	dev_dbg(&pdev->dev,"ioremap() ok, ptr base is = %px\n",addr);
	
	u32 offset = 13;
	u32 __iomem * offset_addr = (u32 __iomem *) addr + offset;
	

	//unsigned long remapped_addr = *((unsigned long *) addr);
	dev_dbg(&pdev->dev,"ioremap() ok, ptr base for GPIO is = %px\n",addr);
	
	dev_dbg(&pdev->dev,"ioremap() ok, ptr base for GPIO get level is = %px\n",offset_addr);
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
	st->base_address = offset_addr;
	/* tied to logic low, analog input range is +/- 5V */
	st->range[0] = 0;
	st->oversampling = 1;
	st->scale_avail = ad7606_scale_avail;
	st->num_scales = ARRAY_SIZE(ad7606_scale_avail);
	st->buffer_enabled = false;
	
	ret = devm_regulator_get_enable(&pdev->dev, "avcc");
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
					"Failed to enable specified AVcc supply\n");
	
	dev_dbg(&pdev->dev,"ad7606:devm_regulator_get_enable() ok\n");
	dev_dbg(&pdev->dev,"chip_id is = %s\n",id->name);
	
	//BUG : the id->driver_data pointer has an issue, seems unable to fetch from the dts, or a struct/pointer nesting issue.
	// WORKAROUND : hardcode id driver_data to 1 = ID_AD7606_8
	st->chip_info = &ad7606_chip_info_tbl[id->driver_data];

	//st->chip_info = &ad7606_chip_info_tbl[ID_AD7606_8];


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
	//indio_dev->modes = INDIO_DIRECT_MODE; - no trigger support
	indio_dev->modes = INDIO_BUFFER_TRIGGERED; //- modern trigger support, TODO : update to modern iio calls to use it. 
	//indio_dev->modes = INDIO_RING_TRIGGERED; // legacy trigger support .
	

	indio_dev->name = id->name;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	
	init_completion(&st->completion);
	
	ret = ad7606_reset(st); // a reset is required after power up to ensure correct RANGE
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



	/*
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
	*/

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


	
	#if DEBUG
	for(u16 i = 0;i<256;i++)
	{ 
		u32 testval = readl(addr);
		dev_dbg(&pdev->dev,"readl() on GPIO memory map test : ptr = %px\n",addr);
		dev_dbg(&pdev->dev,"readl() on GPIO memory map test : val = %u\n",testval);
		addr = (u32 __iomem *) addr + 1;
	}
	#endif

	return devm_iio_device_register(&pdev->dev, indio_dev);
		

}

static int __init board_init(void)
{
    pr_debug("ad7606_par module init.");
	
	//platform_add_devices(board_devices, ARRAY_SIZE(board_devices));
	pr_debug("platform_add_devices() called.");
	platform_driver_register(&ad7606_driver);
	pr_debug("platform_driver_register() called.");

    return 0;
}

static void board_unload(void)
{
    pr_debug("ad7606_par module unload.");
	//struct iio_dev *indio_dev = dev_get_drvdata(&_pdev->dev);
	//devm_iio_device_unregister(&_pdev->dev, indio_dev);
	//pr_debug("devm_iio_device_unregister() called.");
	
	//devm_iio_device_free(indio_dev);
	//pr_debug("devm_iio_device_free() called.");
	
	//platform_device_unregister(_pdev);
	//pr_debug("platform_device_unregister() called.");


	platform_driver_unregister(&ad7606_driver);
	pr_debug("platform_driver_unregister() called.");
	//iounmap(addr);
	//release_mem_region(phy_addr,SZ_4K);
	pr_debug("module unloaded sucessfully.");
	
}

//arch_initcall(board_init);
module_init(board_init);
module_exit(board_unload);

//module_platform_driver(ad7606_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606 ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_AD7606);
