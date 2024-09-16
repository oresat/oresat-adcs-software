
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


    # get old packets
    with open('packets.json', 'r') as packet_file:
        saved_packets = json.load(packet_file)

    # get new packets
    packet_url = "https://db.satnogs.org/api/telemetry/?format=json&sat_id=DKCD-1609-0567-7056-3922"





    num_new_packets = 0
    all_new_packets = True
    while all_new_packets and num_new_packets < 40:
        # sleep
        time.sleep(2)
        print("getting more packets...")
        # get next packet
        packet_set = get_packet_set(auth, packet_url)
        print(packet_set)

        # assumes packets are sorted by timestamp
        for packet in packet_set['results']:
            if packet['timestamp'] in saved_packets.keys():
                print(packet['timestamp'], saved_packets.keys())
                # break out of the loop
                all_new_packets = False
                break
            saved_packets[str(packet['timestamp'])] = str(packet['frame'])
            num_new_packets += 1

        packet_url = packet_set['next']


    if num_new_packets > 0:
        with open('new_packets.json', 'w') as new_packet_file:
            json.dump(saved_packets, new_packet_file, indent=4)

        print("if new_packets.json looks good, then move it to packets.json")




