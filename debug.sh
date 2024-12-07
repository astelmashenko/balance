#!/bin/bash

# linux
#gdb-multiarch -q -ex "target remote :3333" target/thumbv7m-none-eabi/debug/balance

# macos
arm-none-eabi-gdb -q -ex "target remote :3333" target/thumbv7m-none-eabi/debug/balance

