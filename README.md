# ad7606_par_rpi
Analog Devices ad7606 IIO driver module for parallel interface mode on Raspberry Pi

LICENSE INFORMATION :

// SPDX-License-Identifier: GPL-2.0
/*
 * AD7606 Parallel Interface ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Notice : Derivative Work :
 *
 * Copyright 2025 SKYNEXT Rodrigo Verissimo EURL.



 *
 * Raspberry Pi ad7606 IIO driver module  parallel interface implementation, with CS/RD pin support for reading all channels

 * DESIGN GOALS :
 * This work is in proof of concept stage, with the main goal being to test it works, and the limits of sampling rate on such a SoC.
 * This is a "no backend" version, that is, no IC/ ADI IP core, or any device between the AD7606 and the raspberry pi is required to drive the parallel interface and the IC. The downside being a limited sampling rate, jitter /potential
non equispaced sampling due to kernel preemption and scheduling constraints.

 * 
 * Implementing as a loadable module (modprobe / insmod)

 * Integration of the GPIO MMIO address space into 'reg' property of the device tree instead of board support code (to avoid a mixup of board support + device * tree)

 * Adding support for byte shifting to extract conversion results on GPIO BCM (1 byte shift)
 * on BCM pins GPIO8 to GPIO23 as parallel interface pins, while maintaining memory aligment constrains (4 byte alignment, 4 byte read i/o)
 * Note we are not far from using ALL GPIO pins on a Raspberry pi, so disabling inboard peripherals and their corresponding GPIOs is required :
 * no I2C, SPI or UART, nor EEPROM HAT support, as to get control over all GPIO pins without conflicts.
 
 * Adding driver unload support (exit_module() macro)
 * Testing in progress on Raspberry Pi Zero W 1.1
 * Added debug logging
 *
 */

Driver as well as device tree overlay compile and load properly
WORK in progress, not tested with ad7606 hardware yet.

The derivative work is based on the Analog Devices page :
https://developer.analog.com/software/drivers/linux/ad7606



dependency :

install your current kernel linux-headers with apt
use raspi-config to disable ALL peripherals : I2C, SPI, UART, 
for EEPROM HAT reserved pins : 

force_eeprom_read=0 in config.txt to free GPIO0 and GPIO1

to compile :

make

to load module :

chmod +x load.sh
./load.sh

load.sh description :

sudo modprobe industrialio # loads industrialio, as the driver depends on it
sudo modprobe industrialio_triggered_buffer # loads industrialio_triggered_buffer, as the driver depends on it
sudo dtoverlay -v ad7606.dtbo # loads the compiled ad7606 device tree overlay.
sudo insmod  ad7606_par.ko # load the compiled ad7606_par driver module
sudo vclog -m # debug output for the overlay load operation.

to check for proper module loading, run :

dmesg


TODO : CS/RD pin support to shift AD7606 conversion register as to read more than one channel.
TODO : fix id->driver_data fetch from dts so as to get the proper ic. for now, hardcoded to AD7606-8
TODO : test non triggered read on all channels
TODO : test insw() and byte shifting to read gpio pins 8 to 23 / alternatively use ioread32()
TODO : test from userspace using industrialio exposed device.
TODO : test triggered mode and buffer read for all channels
TODO : test oversampling, 
TODO : convst a / convst b to trigger conversion for the first 4 channels and last 4 channels separately
TODO : conformity testing with the signalling protocol exposed in the datasheet.
TODO : conversion timestamping
