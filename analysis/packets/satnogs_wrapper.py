
import requests
import json
import os
import time



def read_satnogs_auth():
    # Open your auth key and print it
    abs_path = os.path.abspath(__file__)
    parent_path = os.path.dirname(abs_path)
    parent_parent_path = os.path.dirname(parent_path)
    with open(os.path.join(parent_parent_path, 'satnogs-api-key.txt'), 'r') as fd:
        auth = fd.readline().strip()

    return auth


def get_packet_set(auth_key, packet_url):
    headers = {'Accept': 'application/json', 'Authorization': f'Token {auth_key}'}
    result = requests.get(packet_url, headers=headers)
    result = json.loads(result.text)
    return result



def get_new_telemetry(auth, page_url, delay=10, stop_match=None):
    '''
        auth (str): satnogs auth token
        page_url (str): initial telemetry url
        delay (float): the seconds to sleep between each request, default 10s
        early_stop (str): if there was a previous session, stop when the response matches this string. This is normally the most recent response in the cache.
    '''
    new_telemetry = list()
    
    # get new packets
    # page_url = "https://db.satnogs.org/api/telemetry/?format=json&sat_id=DKCD-1609-0567-7056-3922"

    while page_url is not None:
        try:
            response = get_packet_set(auth, page_url)

            for packet in response['results']:
                hey = json.dumps(packet)
                #print(hey)

                if stop_match is not None and hey == stop_match:
                    page_url = None
                    # throw an error
                    raise StopIteration

                new_telemetry.append(hey)

            # save intermittently
            with open('prepended_raw.json', 'w') as telemetry_file:
                json.dump(new_telemetry, telemetry_file, indent=4)

            # don't throttle the satnogs api
            time.sleep(delay)
            page_url = response['next']
            print('Next page url:', page_url)


        except (StopIteration) as e:
            print('\n\nPacket matches the early stop string')

        #except Exception as e:
        #    print('\n\nError:', e)
        #    print('\n\nResponse', response)
        #    break

    return new_telemetry




