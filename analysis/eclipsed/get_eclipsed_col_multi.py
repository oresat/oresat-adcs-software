from datetime import datetime
import ephem
import csv

if __name__ == "__main__":
    my_TLEs = [["1 98867U          24238.20000000  .00000000  00000-0  20199-3 0    06", "2 98867  97.4404 314.0487 0008202 330.6740  42.2938 15.18964090    01"],
               ["1 98867U          24232.66241898  .00000000  00000-0  20199-3 0    02", "2 98867  97.4404 307.5566 0009895 130.8460 221.6340 15.18810746    02"]]

    orbits = [ephem.readtle("ORESAT0.5 (ORESAT0.5)", tle_set[0], tle_set[1]) for tle_set in my_TLEs]
    orbit_epochs = [datetime.strptime(str(orbit.epoch), "%Y/%m/%d %H:%M:%S") for orbit in orbits]
    #print(orbit_epochs)
    # USE PYEPHEM


    new_csv_data = []
    filename = "Beacon Data CSV 2024-08-29 10_00 - Beacons.csv"
    with open("Beacon Data CSV 2024-08-29 10_00 - Beacons.csv", "r") as fd:
        reader = csv.reader(fd)
        for i,line in enumerate(reader):
            if i == 0:
                content = "is_eclipsed"
            else:
                # for each orbit

                date = datetime.strptime(line[0], "%Y-%m-%dT%H:%M:%SZ")
                delta_t = [abs((date - odate).total_seconds()) for odate in orbit_epochs] 
                the_index = delta_t.index(min(delta_t))
                print(the_index)
                # find the index of the closest TLE
                

                # compare all orbits for the best
                
                content = ""
                #sat_ephem.compute((line[0]).replace("T", " ").replace("Z", " "))
                #content = str(sat_ephem.eclipsed)
            new_csv_data.append([content])

            if i > 10:
                break


    save_filename = "Better IsEclipsed " + filename
    with open(save_filename, "w", newline='') as fd:
        writer = csv.writer(fd)
        writer.writerows(new_csv_data)

    for line in new_csv_data:
        print(line)

    