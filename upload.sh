#!/bin/bash
cargo build --release
cargo objcopy --release -- -O binary greenhouse.bin
"/home/hex/.arduino15/packages/adafruit/tools/bossac/1.8.0-48-gb176eee/bossac" -i -d --port=ttyACM0 -U -i --offset=0x2000 -w -v "/home/hex/Documents/GitHub/greenhouse/greenhouse.bin" -R
