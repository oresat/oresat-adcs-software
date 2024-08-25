

if __name__ == "__main__":

    main_TLE_1="1 70312C 24149M   24229.82868808  .00004623  00000+0  22387-3 0"
    main_TLE_2="2 98867  97.4404 307.5566 0009895 130.8460 221.6340 15.18810746"

    main_TLE_1="1 70313C 24149M   24232.66241898  .00000000  00000-0  20199-3 0"
    main_TLE_2="2 70313  97.4404 307.5566 0009895 130.8460 221.6340 15.18810746"


    total = 0
    for char in main_TLE_1:
        if char.isdigit():
            #print(char, int(char))
            total += int(char)

        if char == "-":
            #print(char, "1")
            total+=1

    print(total%10)



    total = 0
    for char in main_TLE_2:
        if char.isdigit():
            #print(char, int(char))
            total += int(char)

        if char == "-":
            #print(char, "1")
            total+=1

    print(total%10)
