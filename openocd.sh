#!/bin/bash

openocd -f interface/stlink.cfg -f target/stm32f1x.cfg

#rustup target add thumbv7m-none-eabi
#gdb-multiarch -q -ex "target remote :3333" .pio/build/bluepill_f103c8/firmware.elf
#cargo readobj --target thumbv7m-none-eabi --bin led-roulette -- --file-header
#
