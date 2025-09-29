#!/usr/bin/python
#
# File	: gen_appbin.py
# This file is part of Espressif's generate bin script.
# Copyright (C) 2013 - 2016, Espressif Systems
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of version 3 of the GNU General Public License as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

"""This file is part of Espressif's generate bin script.
    argv[1] is elf file name
    argv[2] is version num"""

from __future__ import print_function

import string
import sys
import os
import re
import binascii
import struct


TEXT_ADDRESS = 0x40108000
# app_entry = 0
# data_address = 0x3ffb0000
# data_end  = 0x40000000
# text_end  = 0x40120000

CHECKSUM_INIT = 0xEF

chk_sum = CHECKSUM_INIT
blocks = 0
PY3 = sys.version_info[0] == 3

def write_file(file_name,data):
	if file_name is None:
		print('file_name cannot be none\n')
		sys.exit(0)

	fp = open(file_name,'ab')

	if fp:
		fp.seek(0,os.SEEK_END)
		fp.write(data)
		fp.close()
	else:
		print('%s write fail\n'%(file_name))

def combine_bin(file_name,dest_file_name,start_offset_addr,need_chk):
    global chk_sum
    global blocks
    if dest_file_name is None:
        print('dest_file_name cannot be none\n')
        sys.exit(0)

    if file_name:
        fp = open(file_name,'rb')
        if fp:
            ########## write text ##########
            fp.seek(0,os.SEEK_END)
            data_len = fp.tell()
            if data_len:
                if need_chk:
                    tmp_len = (data_len + 3) & (~3)
                else:
                    tmp_len = (data_len + 15) & (~15)
                data_bin = struct.pack('<II',start_offset_addr,tmp_len)
                write_file(dest_file_name,data_bin)
                fp.seek(0,os.SEEK_SET)
                data_bin = fp.read(data_len)
                write_file(dest_file_name,data_bin)
                if need_chk:
                    if need_chk:
                        for byte_val in data_bin:
                            if PY3:
                                chk_sum ^= byte_val
                            else:
                                chk_sum ^= ord(byte_val)
                print('%s size is %d(0x%x),align 4 bytes,\nultimate size is %d(0x%x)'%(file_name,data_len,data_len,tmp_len,tmp_len))
                tmp_len = tmp_len - data_len
                if tmp_len:
                    data_str = ['00']*(tmp_len)
                    data_bin = binascii.a2b_hex(''.join(data_str))
                    write_file(dest_file_name,data_bin)
                    if need_chk:
                        for byte_val in data_bin:
                            if PY3:
                                chk_sum ^= byte_val
                            else:
                                chk_sum ^= ord(byte_val)
                blocks = blocks + 1
                fp.close()
        else:
        	    print('!!!Open %s fail!!!'%(file_name))

def gen_appbin():
    global chk_sum
    global blocks
    if len(sys.argv) != 3:
        print('Usage: gen_appbin.py eagle.app.out mode')
        sys.exit(0)

    elf_file = sys.argv[1]
    mode = sys.argv[2]

    flash_data_line  = 16
    data_line_bits = 0xf

    irom0text_bin_name = 'eagle.app.v6.irom0text.bin'
    text_bin_name = 'eagle.app.v6.text.bin'
    data_bin_name = 'eagle.app.v6.data.bin'
    rodata_bin_name = 'eagle.app.v6.rodata.bin'
    flash_bin_name ='eagle.app.flash.bin'

    BIN_MAGIC_FLASH  = 0xE9
    BIN_MAGIC_IROM   = 0xEA
    data_str = ''
    sum_size = 0

    cmd = 'xtensa-lx106-elf-nm -g ' + elf_file + ' > eagle.app.sym'
    os.system(cmd)

    fp = file('./eagle.app.sym')
    if fp is None:
        print("open sym file error\n")
        sys.exit(0)

    lines = fp.readlines()
    fp.close()

    entry_addr = None
    p = re.compile('(\\w*)(\\sT\\s)(call_user_start1)$')
    for line in lines:
        m = p.search(line)
        if m != None:
            entry_addr = m.group(1)
            print(entry_addr)

    if entry_addr is None:
        print('no entry point!!')
        sys.exit(0)

    data_start_addr = '0'
    p = re.compile('(\\w*)(\\sA\\s)(_data_start)$')
    for line in lines:
        m = p.search(line)
        if m != None:
            data_start_addr = m.group(1)
            print(data_start_addr)

    rodata_start_addr = '0'
    p = re.compile('(\\w*)(\\sA\\s)(_rodata_start)$')
    for line in lines:
        m = p.search(line)
        if m != None:
            rodata_start_addr = m.group(1)
            print(rodata_start_addr)
    
    if mode == '2':
        # write irom bin head
        data_bin = struct.pack('<BBHI',BIN_MAGIC_IROM,4,0,int(entry_addr,16))
        sum_size = len(data_bin)
        write_file(flash_bin_name,data_bin)
        
        # irom0.text.bin
        combine_bin(irom0text_bin_name,flash_bin_name,0x0,0)

    # write flash bin header
    data_bin = struct.pack('<BBHI',BIN_MAGIC_FLASH,3,0,int(entry_addr,16))
    sum_size = len(data_bin)
    write_file(flash_bin_name,data_bin)

    # text.bin
    combine_bin(text_bin_name,flash_bin_name,TEXT_ADDRESS,1)

	# data.bin
    if data_start_addr:
        combine_bin(data_bin_name,flash_bin_name,int(data_start_addr,16),1)

	# rodata.bin
    combine_bin(rodata_bin_name,flash_bin_name,int(rodata_start_addr,16),1)

    # write checksum header
    sum_size = os.path.getsize(flash_bin_name) + 1
    sum_size = flash_data_line - (data_line_bits&sum_size)
    if sum_size:
        data_str = ['00']*(sum_size)
        data_bin = binascii.a2b_hex(''.join(data_str))
        write_file(flash_bin_name,data_bin)
    write_file(flash_bin_name,bytearray([chk_sum & 0xFF]))
    	
    if mode == '1':
        sum_size = os.path.getsize(flash_bin_name)
        data_str = ['FF']*(0x10000-sum_size)
        data_bin = binascii.a2b_hex(''.join(data_str))
        write_file(flash_bin_name,data_bin)

        fp = open(irom0text_bin_name,'rb')
        if fp:
            data_bin = fp.read()
            write_file(flash_bin_name,data_bin)
            fp.close()
        else :
            print('!!!Open %s fail!!!'%(flash_bin_name))
            sys.exit(0)

    cmd = 'rm eagle.app.sym'
    os.system(cmd)

if __name__=='__main__':
    gen_appbin()
