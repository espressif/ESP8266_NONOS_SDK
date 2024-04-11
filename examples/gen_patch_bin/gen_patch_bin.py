import os

if os.path.exists("./patch_array.h"):
    os.remove("./patch_array.h")

if os.system("./gen_misc.sh") != 0:
    print("Run gen_misc.sh failed")
    exit(-1)

if os.system("./gen_bin.sh") != 0:
    print("Run gen_bin.sh failed")
    exit(-1)

with open("bin/eagle.app.v6.flash.bin","r") as fd:
    with open("./patch_array.h","w+") as patch_fd:
        patch_fd.write("// This file is a patch for th25q16\r\n\r\n")
        patch_fd.write("#include \"os_type.h\"\r\n\r\n")
        patch_fd.write("const uint8 patch_data[] STORE_ATTR = {\r\n")
        mystr = fd.read()
        len1 = len(mystr)
        Hex_Str = ""
        offset = 0
        for i in range(0, len1,1):
            Hex_Str += "0x"+(hex(ord(mystr[i])).replace('0x','').zfill(2)).upper()+ ","
            offset += 1
            if offset == 16:
                print Hex_Str
                patch_fd.write("    {}\r\n".format(Hex_Str))
                Hex_Str = ""
                offset = 0
        
        if Hex_Str != "":
            print Hex_Str
            patch_fd.write("    {}\r\n".format(Hex_Str))
        patch_fd.write("};\r\n")

