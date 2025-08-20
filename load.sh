#!/bin/bash
sudo modprobe industrialio # load industrial io kernel module
sudo modprobe iio-trig-hrtimer # load hrtimer industrial io kernel module
sudo modprobe industrialio_triggered_buffer # load industrial triggered buffer kernel module
sudo mkdir /sys/kernel/config/iio/triggers/hrtimer/hrtimer0 # configure one hrtimer instance
#sudo dtoverlay -v frstdatapin.dtbo #load compiled device tree snippet to enforce

sudo pinctrl -e -v set 4 ip pd
sudo pinctrl -e -v set 7 ip pd

sudo pinctrl -e -v set 8-23 ip pd

# frstdata pin PULL_DOWN state
sudo dtoverlay -v ad7606.dtbo #load compiled device tree snippet with ad7606 pin to rpi config
sudo insmod  ad7606_par.ko #load ad7606 driver module
sudo sh -c "echo 10.0 > /sys/bus/iio/devices/trigger0/sampling_frequency" #sets triggered sampling frequency to 10 sps
sudo sh -c "cat /sys/bus/iio/devices/trigger0/name > /sys/bus/iio/devices/iio\:device0/trigger/current_trigger" # configure the ad7606 iio device to use the hrtimer0 instance
sudo sh -c "echo 1 > /sys/bus/iio/devices/iio\:device0/scan_elements/in_voltage5_en" # configure the triggered buffer to output a single channel, the second channel. (channels are 0 indexed)
cat /sys/bus/iio/devices/iio\:device0/in_voltage1_raw #output raw value for testing. The first sample is usually invalid, this needs debugging.
cat /sys/bus/iio/devices/iio\:device0/in_voltage1_raw #gets another sample, this value should be ok.
#sudo vclog -m


#Make sure tracing is disabled during tracing reconfiguration
echo "disabling tracing and current_tracer"
#sudo sh -c "echo 0 > /sys/kernel/debug/tracing/tracing_on"
#sudo sh -c "echo nop > /sys/kernel/debug/tracing/current_tracer"


#sudo sh -c "echo ad7606_* > /sys/kernel/debug/tracing/set_ftrace_filter"
echo "ad7606_* written to set_ftrace_filter"
#sudo sh -c "echo function_graph > /sys/kernel/debug/tracing/current_tracer"
#sudo sh -c "echo 2 > /sys/kernel/debug/tracing/max_graph_depth"
echo "set max_graph_depth to 2"

#sudo sh -c "echo 1 > /sys/kernel/debug/tracing/tracing_on"
echo "function_graph enabled in current_tracer, and enabling tracing "

echo "now enabling device buffer"
sudo sh -c 'echo 1 > /sys/bus/iio/devices/iio\:device0/buffer0/enable'
echo "buffer enabled"

#echo "reading trace_pipe in 2 secs"
#sleep 2
#sudo sh -c "cat /sys/kernel/debug/tracing/trace_pipe > /root/trace.log"

echo "will now output raw, aligned sample bytes from buffer device"
sudo sh -c "dd if=/dev/iio\:device0 bs=20 iflag=fullblock | hexdump"
sudo pinctrl -e -v set 7 ip pd
