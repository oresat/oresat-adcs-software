
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

    print("This script may not work, please edit this file if you wish to test it")
    exit()


    # Open your auth key and print it
    abs_path = os.path.abspath(__file__)
    parent_path = os.path.dirname(abs_path)
    parent_parent_path = os.path.dirname(parent_path)
    with open(os.path.join(parent_parent_path, 'satnogs-api-key.txt'), 'r') as fd:
        auth = fd.readline().strip()
        print(auth)


    # Load old telemetry
    with open('ref_telemetry.json', 'r') as fd:
        ref_telemetry = json.load(fd)


    new_telemetry = list()

    # get new packets
    page_url = "https://db.satnogs.org/api/telemetry/?format=json&sat_id=DKCD-1609-0567-7056-3922"
    while page_url is not None:
        try:
            response = get_packet_set(auth, page_url)

            for packet in response['results']:
                hey = json.dumps(packet)

                # if this telemetry is the same as the beginning of the saved
                # reference telemetry, stop
                if hey == ref_telemetry[0]:
                    page_url = None
                    # throw an error
                    raise StopIteration

                new_telemetry.append(hey)

            # save intermittently
            with open('prepended_raw.json', 'w') as telemetry_file:
                json.dump(new_telemetry, telemetry_file, indent=4)

            # don't throttle the satnogs api
            time.sleep(10)
            page_url = response['next']
            print(page_url)

        except (StopIteration) as e:
            print('Iteration has ended')

        except Exception as e:
            print('Error:', e)
            print('Response', response)
            break

    # save one more time for safety
    with open('prepended_raw_finished.json', 'w') as telemetry_file:
        json.dump(new_telemetry.extend(ref_telemetry), telemetry_file, indent=4)




