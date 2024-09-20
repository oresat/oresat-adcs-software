

import zlib
import canopen
import json

from oresat_configs import OreSatConfig, OreSatId


DATA_TYPE_SIZE = {
    canopen.objectdictionary.datatypes.BOOLEAN: 1,
    canopen.objectdictionary.datatypes.INTEGER8: 1,
    canopen.objectdictionary.datatypes.INTEGER16: 2,
    canopen.objectdictionary.datatypes.INTEGER32: 4,
    canopen.objectdictionary.datatypes.UNSIGNED8: 1,
    canopen.objectdictionary.datatypes.UNSIGNED16: 2,
    canopen.objectdictionary.datatypes.UNSIGNED32: 4,
    canopen.objectdictionary.datatypes.REAL32: 4,
    canopen.objectdictionary.datatypes.REAL64: 8,
    canopen.objectdictionary.datatypes.INTEGER64: 8,
    canopen.objectdictionary.datatypes.UNSIGNED64: 8,
}

beacon_def = OreSatConfig(OreSatId.ORESAT0_5).beacon_def

data = []

if __name__ == "__main__":
    
    with open('ref_raw.json', 'r') as fd:
        ref_raw = json.load(fd)


    i = 0
    for packet in ref_raw:
        i += 1
        # based on https://github.com/oresat/oresat-random-scripts/blob/master/satnogs_fetch.py

        # packets were in json format saved in strings
        ref_dict = json.loads(packet)
        print(ref_dict)
        
        frame = ref_dict['frame']

        msg = bytes.fromhex(frame)
        crc32_calc = zlib.crc32(msg[16:-4], 0).to_bytes(4, "little")

        if crc32_calc != msg[-4:]:
            print("invalid crc32\n")
            continue


        row = ''
        offset = 16  # skip ax25 header
        for obj in beacon_def:
            size = DATA_TYPE_SIZE.get(obj.data_type, 0)
            if size == 0:
                size = len(obj.value)
            value = obj.decode_raw(msg[offset : offset + size])
            if obj.bit_definitions:
                for i in obj.bit_definitions.values():
                    row += f"{bool(value & (1 << i))},"
            else:
                value = obj.value_descriptions.get(value, value)
                row += f"{value},"
            offset += size
        row += f'{int.from_bytes(msg[-4:], "little")}\n'

        print(row)

        if i > 2:
            break
