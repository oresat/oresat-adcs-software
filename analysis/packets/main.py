
import os
import json
import satnogs_wrapper as satnogs
import oresat_decoder as oresat

if __name__ == "__main__":

    auth = satnogs.read_satnogs_auth()
    page_url = "https://db.satnogs.org/api/telemetry/?format=json&sat_id=DKCD-1609-0567-7056-3922"

    saved_cache_filename = 'saved_raw_cache.json'
    latest_response = None
    updated_cache_filename = 'init_raw_cache.json'


    if os.path.isfile(os.path.abspath(saved_cache_filename)):
        print("cache exists")
        with open(saved_cache_filename, 'r') as fd:
            old_telemetry = json.load(fd)
        latest_response = old_telemetry[0]
        new_telemetry = satnogs.get_new_telemetry(auth, page_url, delay=10, stop_match=latest_response)
        all_telemetry = new_telemetry + old_telemetry
        with open('updated_raw_cache.json', 'w') as fd:
            json.dump(all_telemetry, fd, indent=4)
        print("\n\nNew cache named `updated_raw_cache.json` created. Please check it and rename to `saved_raw_cache.json`")
    else:
        print("cache does not exist")
        all_telemetry = satnogs.get_new_telemetry(auth, page_url, delay=10)
        with open('init_raw_cache.json', 'w') as fd:
            json.dump(all_telemetry, fd, indent=4)

        print("\n\nNew cache named `init_raw_cache.json` created. Please check it and rename to `saved_raw_cache.json`")



    # at this point all_telemetry has everything we want
    decoded_packets = oresat.decode_responses(all_telemetry)

    with open('decoded_beacon_cache.json', 'w') as fd:
        json.dump(decoded_packets, fd, indent=4)


    print("\n\nDecoded data saved in cache `decoded_beacon_cache.json`")



