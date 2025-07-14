import mmap
import os
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)


for channel in range(0,28):
    GPIO.setup(channel, GPIO.IN, pull_up_down=GPIO.PUD_UP)
for channel in range(0,28):
    print(str(channel))
    GPIO.setup(channel, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO_BASE = 0x3F200000  # GPIO base address for Raspberry Pi Zero W
GPIO_LEN = 0x1000       # Length of the GPIO memory region

# Open /dev/gpiomem
with open("/dev/gpiomem", "r") as f:
    # Memory-map the GPIO region
    gpio_mem = mmap.mmap(f.fileno(), GPIO_LEN, access=mmap.ACCESS_READ)

    # Example: Read the GPLEV0 register (Offset: 0x34)
    #gpio_mem.seek(0x34)
    #gplev0_0 = gpio_mem.read(1)  # Read 1 byte (8-bit register)
    #gpio_mem.seek(0x35)
    #gplev0_1 = gpio_mem.read(1)  # Read 1 bytes (8-bit register)
    #gpio_mem.seek(0x36)
    #gplev0_2 = gpio_mem.read(1)  # Read 1 bytes (8-bit register)
    #gpio_mem.seek(0x37)
    #gplev0_3 = gpio_mem.read(1)  # Read 1 bytes (8-bit register)
 

    #print("GPLEV0 (byte boundary split) register value (Binary):")
    #print("{:#010b}".format(int.from_bytes(gplev0_0,'little')))
    #print("{:#010b}".format(int.from_bytes(gplev0_1,'little')))
    #print("{:#010b}".format(int.from_bytes(gplev0_2,'little')))
    #print("{:#010b}".format(int.from_bytes(gplev0_3,'little')))

    print(gpio_mem.tell())
    gpio_mem.seek(0x34)
    
    print(gpio_mem.tell())
    gplev0 = gpio_mem.read(4)
    print(gpio_mem.tell())

    gpio_mem.seek(0x35)
    gplev0b = gpio_mem.read(4)  # Read 4 bytes (32-bit register)
    gpio_mem.seek(0x38)
    gplev0c = gpio_mem.read(4)  # Read 4 bytes (32-bit register)
   
    # Print the GPLEV0 value as a hex
    #print(f"GPLEV0 register value: {int.from_bytes(gplev0, 'little'):08X}")
    print("GPLEV0 register value (Binary):")
    print("{:#034b}".format(int.from_bytes(gplev0,'little')))
    print("{:#034b}".format(int.from_bytes(gplev0b,'little')))
    print("{:#034b}".format(int.from_bytes(gplev0c,'little')))
    #print("GPLEV1 register value (Binary):")
    #print("{0:b}".format(int.from_bytes(gplev1,'little')))
    # Close the mmap
    gpio_mem.close()

