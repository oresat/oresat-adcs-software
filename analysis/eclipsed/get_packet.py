
import requests
import json
import os
import sys

def get_packet(auth_key, next_page=None, satnogs_id="DKCD-1609-0567-7056-3922"):
    headers = {'Accept': 'application/json', 'Authorization': f'Token {auth_key}'}

    if next_page is None:
        next_page = f"https://db.satnogs.org/api/telemetry/?format=json&sat_id={satnogs_id}"
    
    result = requests.get(next_page, headers=headers)
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


    #my_result = get_packet(auth)
    

    print(my_result)
