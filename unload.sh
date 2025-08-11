#!/bin/bash
#
sudo sh -c "echo 0 > /sys/bus/iio/devices/iio\:device0/buffer/enable" 
sudo rmmod ad7606_par.ko
