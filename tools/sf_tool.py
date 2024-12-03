#!/usr/bin/env python3
import argparse
import json
import os
import sys


def main():
    args = parse_args()
    print(args.proj_list)
    build_upload_scripts(args.main, args.ptab, args.proj_list, args.output)

def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("main",
                        help="Main project name")
    parser.add_argument("ptab",
                        help="ptab file path")
    parser.add_argument("proj_list",
                        nargs="+",
                        help="Project name list")
    parser.add_argument("output",
                        help="output file path")

    return parser.parse_args()

def init_indent():
    global level
    level = 0

def inc_indent():
    global level
    level += 1


def dec_indent():
    global level
    level -= 1
    if (level < 0):
        level = 0


# return space for indentation
def get_indent_space():
    return ' ' * level * 4


# make one line according to indentation level
def make_lines(s):
    return get_indent_space() + s + '\n'



def build_upload_scripts(main_name, ptab, proj_list, output):

    # proj name as key, if proj has multiple binary, the value is a dictionary too. And binary name is used as key
    # example1:  {"main": 0x18000000}
    # example1:  {"main": {"ROM1.bin": 0x18000000, "ROM2.bin": 0x18200000}}
    img_download_info = {}
    f = open(ptab)
    try:
        mems = json.load(f)
    finally:
        f.close()
    for mem in mems:
        mem_base = int(mem['base'], 0)
        for region in mem['regions']:
            offset = int(region['offset'], 0)
            start_addr = mem_base + offset
            if 'img' in region:
                img_name = region['img']
                img_name = img_name.split(':')
                proj_name = img_name[0]
                if len(img_name) == 1:
                    assert proj_name not in img_download_info, "{} download address already configured".format(proj_name)
                    img_download_info[proj_name] = start_addr
                else:
                    if proj_name not in img_download_info:
                        img_download_info[proj_name] = {}
                    assert img_name[1] not in img_download_info[
                        proj_name], "{} download address already configured".format(region['img'])
                    img_download_info[proj_name][img_name[1]] = start_addr

    init_indent()
    s = ''
    s += make_lines('si SWD')
    s += make_lines('speed 10000')
    s += make_lines('r')
    ImgDownUart_PATH = "ImgDownUart.exe"
    s_file = make_lines('[FILEINFO]')
    s_num = 0
    work_dir = os.path.dirname(output)
    for proj in proj_list:
        if proj in img_download_info:
            # load address is defined in map, load binary using address
            info = img_download_info[proj]
            if proj == 'main':
                bin_file = main_name + '.bin'

            s += make_lines('loadbin {} 0x{:08X}'.format(bin_file, info))

            s_file += make_lines('FILE{}={}'.format(s_num,bin_file))
            s_file += make_lines('ADDR{}=0x{:08X}'.format(s_num,info))
            s_num += 1
        else:
            if proj == 'main':
                hex = main_name + '.hex'
            else:
                hex_file = proj + '.hex'
            # load address is not defined, load hex
            s += make_lines('loadfile {}'.format(hex_file))
            s_file += make_lines('FILE{}={}'.format(s_num,hex_file))
            s_file += make_lines('ADDR{}=0x{:08X}'.format(s_num,0XFFFFFFFF))
            s_num += 1


    s += make_lines('exit')
    f = open(os.path.join(work_dir, 'download.jlink'), 'w')
    f.write(s)
    f.close()

    s_file += make_lines('NUM={}'.format(s_num))
    sf = open(output, 'w')
    sf.write(s_file)
    sf.close()


if __name__ == "__main__":
    main()
