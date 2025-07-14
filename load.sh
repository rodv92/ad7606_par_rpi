#!/bin/bash
sudo modprobe industrialio
sudo modprobe industrialio_triggered_buffer
sudo dtoverlay -v ad7606.dtbo
sudo insmod  ad7606_par.ko
sudo vclog -m

