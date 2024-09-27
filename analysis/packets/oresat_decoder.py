
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




def decode_frame(frame):
        msg = bytes.fromhex(frame)
        crc32_calc = zlib.crc32(msg[16:-4], 0).to_bytes(4, "little")

        if crc32_calc != msg[-4:]:
            #print("invalid crc32\n")
            return False


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

        return row




def decode_responses(ref_raw):
    '''
    Args:
        ref_raw: list of satnogs telemetry responses (strings of json responses)
    '''

    #with open(input_file_name, 'r') as fd:
    #    ref_raw = json.load(fd)


    telemetry = dict()
    for packet in ref_raw:
        # based on https://github.com/oresat/oresat-random-scripts/blob/master/satnogs_fetch.py

        # packets were in json format saved in strings
        ref_dict = json.loads(packet)
        #print(ref_dict)
        
        frame = ref_dict['frame']
        
        row = decode_frame(frame)
        
        if type(row) is str:
            # print(ref_dict['timestamp'], row)
            telemetry[ref_dict['timestamp']] = row
        else:
            #print("\tIncorrect crc32")
            pass

    return telemetry



