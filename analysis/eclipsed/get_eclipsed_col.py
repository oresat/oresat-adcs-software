
import ephem
import csv

if __name__ == "__main__":
    
    tle1 = "1 98867U          24238.20000000  .00000000  00000-0  20199-3 0    06"
    tle2 = "2 98867  97.4404 314.0487 0008202 330.6740  42.2938 15.18964090    01"

    # USE PYEPHEM
    sat_ephem = ephem.readtle("ORESAT0.5 (ORESAT0.5)", tle1, tle2)

    new_csv_data = []
    filename = "Beacon Data CSV 2024-08-29 10_00 - Beacons.csv"
    with open("Beacon Data CSV 2024-08-29 10_00 - Beacons.csv", "r") as fd:
        reader = csv.reader(fd)
        for i,line in enumerate(reader):
            if i == 0:
                content = "is_eclipsed"
            else:
                sat_ephem.compute((line[0]).replace("T", " ").replace("Z", " "))
                content = str(sat_ephem.eclipsed)
            new_csv_data.append([content])


    save_filename = "IsEclipsed " + filename
    with open(save_filename, "w", newline='') as fd:
        writer = csv.writer(fd)
        writer.writerows(new_csv_data)

    for line in new_csv_data:
        print(line)

    