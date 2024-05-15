#!/bin/bash
arm-none-eabi-objcopy -O ihex build/turtle_bot.elf build/turtle_bot.bin
st-flash write build/turtle_bot.bin 0x8000000b
