
import datetime
import time
from oresat_adcs.classes import jday


if __name__ == "__main__":
    print("Testing jday package")

    print("Checking jclock objects")

    my_jclock = jday.JClock(2023, 1, 1, 1, 1, 1)
    my_jclock_2 = jday.JClock(2024, 1, 1, 1, 1, 1)
    my_jclock_3 = jday.JClock(2024, 1, 1, 1, 16, 1)
    my_jclock_4 = jday.JClock(2025, 1, 1, 1, 1, 1)
    
    print("Asserting the `is_after` function works")
    assert my_jclock_2.is_after(my_jclock.datetime)
    assert not my_jclock.is_after(my_jclock_2.datetime)
    assert not my_jclock.is_after(my_jclock.datetime)

    print("Asserting the `is_minutes_away` function works")
    assert not my_jclock_2.is_minutes_away(my_jclock_3.datetime, 14.0)
    assert my_jclock_2.is_minutes_away(my_jclock_3.datetime, 14.00000001)
    assert my_jclock_2.is_minutes_away(my_jclock_3.datetime, 15.0)
    assert my_jclock_2.is_minutes_away(my_jclock_3.datetime, 15.99999999)
    assert not my_jclock_2.is_minutes_away(my_jclock_3.datetime, 16.0)

    print("Checking the minutes away function")
    assert my_jclock.minutes_away(my_jclock_2.datetime) == 525600 # one year (exactly 365 days) has 525600 seconds
    assert my_jclock_2.minutes_away(my_jclock_4.datetime) == 527040 # one leap year (exactly 366 days) has 527040 seconds


    def test_gmst(year, month, day, hour, minute, second):
        test_jday = jday.Clock(year=year, month=month, day=day, hour=hour, minute=minute, second=second)
        test_jclock = jday.JClock(year=year, month=month, day=day, hour=hour, minute=minute, second=second)

        print("Julian Date: ", test_jday.julian_date(hour, minute, second))
        print("Julian Date: ", test_jclock.julian_date())

        print("Days: ", test_jday.centuries_elapsed()*36525)
        print("Days: ", test_jclock.centuries_elapsed()*36525)

        jday_gmst = test_jday.gmst_seconds()
        jclock_gmst = test_jclock.gmst_seconds()

        print("GMST: ", jday_gmst%86400//3600, jday_gmst%3600//60, jday_gmst%60)
        print("GMST: ", jclock_gmst%86400//3600, jclock_gmst%3600//60, jclock_gmst%60)

        print("Theta: ", test_jday.theta_gmst())
        print("Theta: ", test_jclock.theta_gmst())

    
    print("\nJDActual: 2460498.5833333")
    test_gmst(2024, 7, 7, 2, 0, 0)
    print("GMST Actual: 21:02:08.7500\n")
    
    print("\nJD Actual: 2460498.8333333")
    test_gmst(2024, 7, 7, 8, 0, 0)
    print("GMST Actual: 03:03:07.8888\n")
    
    print("\nJD Actual: 2460499.0833333")
    test_gmst(2024, 7, 7, 14, 0, 0)
    print("GMST Actual: 09:04:07.0277\n")
    
    print("\nJD Actual: 2460499.3333333")
    test_gmst(2024, 7, 7, 20, 0, 0)
    print("GMST Actual: 15:05:06.1665\n")
    

    print("\n\nConfirming clock sync")
    my_jclock_2.sync()
    time.sleep(5)
    assert abs(my_jclock_2.sync() - 5) < 0.001


    print("Done")

