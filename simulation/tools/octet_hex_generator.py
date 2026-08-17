

import struct
import io

def decode_hex(hex_str, data_format='>f'):
    # an example of a hex string is "0123456789abcdef"
    # turn hex string into bytes object
    octet_bytes = bytes.fromhex(hex_str)

    # unpack each one and remove from tuple
    values = [
        value 
        for value_tuple in struct.iter_unpack(data_format, octet_bytes) 
        for value in list(value_tuple)
    ]

    return values


def encode_hex(float_list, data_format='>f'):
    return bytes.hex(b''.join([struct.pack(data_format, val) for val in float_list]))


if __name__ == "__main__":
    do_example = True
    if do_example:
        print("The following is an example:")
        hex_str = "3c872ffa36ee926f383eb70a36ee926f3c82c2bd38025129383eb70a380251293bd5961f"

        a,b,c,d,e,f,g,h,i = decode_hex(hex_str)
        my_list = [a, b, c, d, e, f, g, h, i]
        print(f"example packed list: {my_list}")
        print(f"example hex string: {encode_hex(my_list)}")

    # Prism Satellite
    jxx = 0.031
    # xy will have the lowest correlation
    jxy = 5e-6
    # xz and yz will have slightly larger correlations
    jxz = 5e-5
    jyx = jxy
    jyy = 0.025
    jyz = 5e-5
    jzx = jxz
    jzy = jyz
    jzz = 0.012

    print(f"Prism: {encode_hex([jxx, jxy, jxz, jyx, jyy, jyz, jzx, jzy, jzz])}")

    # Beecon Satellite
    jxx = 0.028
    # xy will have the lowest correlation
    jxy = 5e-6
    # xz and yz will have slightly larger correlations
    jxz = 5e-5
    jyx = jxy
    jyy = 0.023
    jyz = 5e-5
    jzx = jxz
    jzy = jyz
    jzz = 0.011

    print(f"Beecon: {encode_hex([jxx, jxy, jxz, jyx, jyy, jyz, jzx, jzy, jzz])}")

