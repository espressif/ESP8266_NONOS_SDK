#!/bin/bash

echo "gen_misc.sh version 20150511"
echo ""

echo "Please follow below steps(1-5) to generate specific bin(s):"
echo "STEP 1: choose boot version(0=boot_v1.1, 1=boot_v1.2+, 2=none)"

boot=none

echo "boot mode: $boot"
echo ""

echo "STEP 2: choose bin generate(0=eagle.flash.bin+eagle.irom0text.bin, 1=user1.bin, 2=user2.bin)"
echo "enter (0/1/2, default 0):"
app=0
echo "generate bin: eagle.flash.bin+eagle.irom0text.bin"
echo ""

echo "STEP 3: choose spi speed(0=20MHz, 1=26.7MHz, 2=40MHz, 3=80MHz)"
echo "enter (0/1/2/3, default 2):"
spi_speed=40

echo "spi speed: $spi_speed MHz"
echo ""

echo "STEP 4: choose spi mode(0=QIO, 1=QOUT, 2=DIO, 3=DOUT)"
echo "enter (0/1/2/3, default 0):"
spi_mode=QIO


echo "spi mode: $spi_mode"
echo ""

echo "STEP 5: choose spi size and map"
echo "    0= 512KB( 256KB+ 256KB)"
echo "    2=1024KB( 512KB+ 512KB)"
echo "    3=2048KB( 512KB+ 512KB)"
echo "    4=4096KB( 512KB+ 512KB)"
echo "    5=2048KB(1024KB+1024KB)"
echo "    6=4096KB(1024KB+1024KB)"
spi_size_map=0
echo "spi size: 512KB"
echo "spi ota map:  256KB + 256KB"

echo ""
echo ""
echo "start..."
echo ""

make COMPILE=gcc BOOT=$boot APP=$app SPI_SPEED=$spi_speed SPI_MODE=$spi_mode SPI_SIZE_MAP=$spi_size_map
