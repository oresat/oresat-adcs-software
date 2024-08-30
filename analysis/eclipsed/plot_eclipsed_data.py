import datetime
import ephem
import csv
import matplotlib.pyplot as plt

if __name__ == "__main__":

    ref_time = datetime.datetime(2024, 8, 16, 20, 6, 28, tzinfo=datetime.UTC)
    tle1 = "1 98867U          24238.20000000  .00000000  00000-0  20199-3 0    06"
    tle2 = "2 98867  97.4404 314.0487 0008202 330.6740  42.2938 15.18964090    01"

    # USE PYEPHEM
    sat_ephem = ephem.readtle("ORESAT0.5 (ORESAT0.5)", tle1, tle2)


    now = datetime.datetime(2024, 8, 16, 20, 6, 28, tzinfo=datetime.UTC)

    in_sun_mission_time = [0]
    in_sun = [0]
    while (datetime.datetime.now(datetime.UTC) - now).total_seconds() > 0:
        now = now + datetime.timedelta(0, 60)
        in_sun_mission_time.append((now - ref_time).total_seconds())
        # calculate if in sun
        sat_ephem.compute(f"{now.year}/{now.month}/{now.day} {now.hour}:{now.minute}:{now.second}")

        in_sun.append(int(not sat_ephem.eclipsed))
        pass

    print(in_sun)


    mission_time = []
    batt_volt = []
    batt_temp = []

    ref_time = datetime.datetime(2024, 8, 16, 20, 6, 28)
    filename = "Beacon Data CSV 2024-08-29 10_00 - Beacons.csv"
    with open("Beacon Data CSV 2024-08-29 10_00 - Beacons.csv", "r") as fd:
        reader = csv.reader(fd)
        for i,line in enumerate(reader):
            if i == 0:
                batt_volt_index = line.index('battery_1_pack_1_vbatt (mV)')
                batt_temp_index = line.index('battery_1_pack_1_temperature (C)')
            else:
                timestamp = datetime.datetime.strptime(line[0], "%Y-%m-%dT%H:%M:%SZ")
                mission_time.append((timestamp - ref_time).total_seconds())
                batt_volt.append(float(line[batt_volt_index]))
                batt_temp.append(float(line[batt_temp_index]))


    fig = plt.figure()

    ax_batt_volt = plt.axes()
    ax_batt_volt.plot(mission_time, batt_volt, marker='o', linestyle='none', color='blue', markersize=5)
    ax_batt_volt.set_ylim([7000, 7500])
    
    ax_sun_volts = ax_batt_volt.twinx()
    ax_sun_volts.fill_between(in_sun_mission_time, in_sun, color='orange', alpha=0.2)
    ax_sun_volts.set_ylim([0, 1])

    plt.show()



    fig = plt.figure()

    ax_batt_volt = plt.axes()
    ax_batt_volt.plot(mission_time, batt_temp, marker='o', linestyle='none', color='red', markersize=5)
    ax_batt_volt.set_ylim([0, 10])
    
    ax_sun_temp = ax_batt_volt.twinx()
    ax_sun_temp.fill_between(in_sun_mission_time, in_sun, color='orange', alpha=0.2)
    ax_sun_temp.set_ylim([0, 1])

    plt.show()