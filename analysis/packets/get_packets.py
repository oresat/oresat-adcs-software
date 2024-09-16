
import time
import requests
import json
import os
import sys

def get_packet_set(auth_key, packet_url):
    headers = {'Accept': 'application/json', 'Authorization': f'Token {auth_key}'}
    result = requests.get(packet_url, headers=headers)
    result = json.loads(result.text)
    return result




if __name__ == "__main__":
    abs_path = os.path.abspath(__file__)
    #print(abs_path)
    parent_path = os.path.dirname(abs_path)
    #print(parent_path)
    parent_parent_path = os.path.dirname(parent_path)

    #print(parent_parent_path)

    with open(os.path.join(parent_parent_path, 'satnogs-api-key.txt'), 'r') as fd:
        auth = fd.readline().strip()
        print(auth)


    collect = ['frame', 'timestamp', 'station_id', 'observation_id']


    with open('telemetry.json', 'r') as fd:
        old_telemetry = json.load(fd)


    new_telemetry = dict()

    # get new packets
    packet_url = "https://db.satnogs.org/api/telemetry/?format=json&sat_id=DKCD-1609-0567-7056-3922"

    all_new = True
    for ii in range(5):
        packet_set = get_packet_set(auth, packet_url)

        for packet in packet_set['results']:
            packet_hash = str(hash(str(packet)))
            # check if packet is already in dictionary
            if packet_hash in old_telemetry.keys():
                all_new = False
                print('existing hash', packet_hash)
                break
            #otherwise add it
            new_telemetry[packet_hash] = {key: packet[key] for key in collect}

        if not all_new:
            break
        time.sleep(1)
        print('hashes', new_telemetry.keys())
        packet_url = packet_set['next']

    with open('updated_telemetry.json', 'w') as telemetry_file:
        json.dump(old_telemetry.extend(new_telemetry), telemetry_file, indent=4)




