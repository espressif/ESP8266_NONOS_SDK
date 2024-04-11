#!/bin/bash

rm -r latest
cp -r .output latest

if [ ! -d "bin" ]; then
    mkdir bin
fi

rm bin/eagle.app.v6.flash.bin bin/eagle.app.v6.irom0text.bin bin/eagle.app.v6.dump bin/eagle.app.v6.S

cd latest/eagle/debug/image

xtensa-lx106-elf-objdump -x -s eagle.app.v6.out > ../../../../bin/eagle.app.v6.dump
xtensa-lx106-elf-objdump -S eagle.app.v6.out > ../../../../bin/eagle.app.v6.S


xtensa-lx106-elf-objcopy --only-section .text -O binary eagle.app.v6.out eagle.app.v6.text.bin
xtensa-lx106-elf-objcopy --only-section .data -O binary eagle.app.v6.out eagle.app.v6.data.bin
xtensa-lx106-elf-objcopy --only-section .rodata -O binary eagle.app.v6.out eagle.app.v6.rodata.bin
xtensa-lx106-elf-objcopy --only-section .irom0.text -O binary eagle.app.v6.out eagle.app.v6.irom0text.bin

python2 ../../../../tools/gen_appbin_user.py eagle.app.v6.out 0
cp eagle.app.v6.irom0text.bin ../../../../bin/
cp eagle.app.flash.bin ../../../../bin/eagle.app.v6.flash.bin

cd ../../../../
