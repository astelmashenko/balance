#!/bin/bash

gdb-multiarch -q -ex "target remote :3333" target/thumbv7m-none-eabi/debug/balance
