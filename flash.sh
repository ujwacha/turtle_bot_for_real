#!/bin/bash
cd build; make; cd ..;
arm-none-eabi-objcopy -O binary -S build/turtle_bot.elf build/turtle_bot.bin
st-flash write build/turtle_bot.bin 0x8000000
