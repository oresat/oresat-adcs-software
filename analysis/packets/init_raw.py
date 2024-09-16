
import hashlib
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
    parent_path = os.path.dirname(abs_path)
    parent_parent_path = os.path.dirname(parent_path)

    with open(os.path.join(parent_parent_path, 'satnogs-api-key.txt'), 'r') as fd:
        auth = fd.readline().strip()
        print(auth)


    new_telemetry = dict()

    # get new packets
    page_url = "https://db.satnogs.org/api/telemetry/?format=json&sat_id=DKCD-1609-0567-7056-3922"
    while page_url is not None:
        try:
            response = get_packet_set(auth, page_url)

            for packet in response['results']:
                packet_hash = (hashlib.md5(str(packet).encode())).hexdigest()
                if packet_hash in new_telemetry.keys():
                    print('oops hash exists:', packet_hash, new_telemetry[packet_hash])
                new_telemetry[packet_hash] = packet

            # save intermittently
            with open('init_raw.json', 'w') as telemetry_file:
                json.dump(new_telemetry, telemetry_file, indent=4)

            # don't throttle the satnogs api
            time.sleep(10)
            page_url = response['next']
            print(page_url)

        except Exception as e:
            print('Error:', e)
            print('Response', response)
            break

    # save one more time for safety
    with open('init_raw_finished.json', 'w') as telemetry_file:
        json.dump(new_telemetry, telemetry_file, indent=4)




