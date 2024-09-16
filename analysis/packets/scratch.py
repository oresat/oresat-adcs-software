
import json
import zlib

if __name__ == "__main__":

    with open('new_packets.jon', 'r') as fd:
        packets = json.load(fd)


    for timestamp, payload in packets.items():
        try:
            print(timestamp, payload)
            payload_bytes = b'{payload}'
            payload_int = int.from_bytes(payload_bytes, 'little')
            decompressed = zlib.decompress(payload_int)
            print(decompressed)
        except Exception as e:
            print("error: ", e)

