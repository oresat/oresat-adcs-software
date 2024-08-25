
import requests
import json


def get_tle(satnogs_id="DKCD-1609-0567-7056-3922"):
    tle_url = f"https://db.satnogs.org/api/tle/?format=json&sat_id={satnogs_id}"
    result = requests.get(tle_url)
    result = json.loads(result.text)[0]
    return result["tle1"] , result["tle2"]


if __name__ == "__main__":
    tle1, tle2 = get_tle()

    print(tle1)
    print(tle2)
