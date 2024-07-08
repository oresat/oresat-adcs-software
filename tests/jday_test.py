
import datetime
from oresat_adcs.classes import jday


if __name__ == "__main__":
    print("Testing jday package")

    print("Creating datetime objects")
    my_datetime = datetime.datetime(year=2023, month=1, day=1, hour=1, minute=1, second=1)
    my_datetime_2 = datetime.datetime(year=2025, month=1, day=1, hour=1, minute=1, second=1)

    print("Creating jday objects")

    my_jday_clock = jday.Clock(year=2024, month=1, day=1, hour=1, minute=1, second=1)
    my_jday_clock_2 = jday.Clock(year=2025, month=1, day=1, hour=1, minute=1, second=1)

    print("Asserting that `is_after` function is working")
    assert my_jday_clock.is_after([2023, 1, 1, 1, 1, 1])
    assert my_jday_clock.is_after([2024, 1, 1, 1, 1, 1])
    assert not my_jday_clock.is_after([2025, 1, 1, 1, 1, 1])


    # assert not my_jday_clock.is_after([2025, 1, 1, 1, 1, 1])

    print("Asserting that `is_minutes_away` function is working")
    for minutes in range(10, 14):
        assert not my_jday_clock.is_minutes_away([2024, 1, 1, 1, 15, 1], minutes)
    
    for minutes in range(14, 16):
        assert my_jday_clock.is_minutes_away([2024, 1, 1, 1, 15, 1], minutes)
    
    for minutes in range(16, 19):
        assert not my_jday_clock.is_minutes_away([2024, 1, 1, 1, 15, 1], minutes)



    # checks if self is a leap year
    print("Asserting that `is_leap_year` function is working")
    assert my_jday_clock.is_leap_year()
    assert not my_jday_clock_2.is_leap_year()

    # check how many days in a month

    # tick

    # calculate the julian date

    # check centuries elapsed

    # gmst seconds

    # theta gmst


    print("Checking jclock objects")

    my_jclock = jday.JClock(2023, 1, 1, 1, 1, 1)
    my_jclock_2 = jday.JClock(2024, 1, 1, 1, 1, 1)
    my_jclock_3 = jday.JClock(2024, 1, 1, 1, 16, 1)
    my_jclock_4 = jday.JClock(2025, 1, 1, 1, 1, 1)
    
    # make sure the `is_after` function works
    assert my_jclock_2.is_after(my_jclock.datetime)
    assert not my_jclock.is_after(my_jclock_2.datetime)
    assert not my_jclock.is_after(my_jclock.datetime)

    # make sure the `is_minutes_away` works
    print(my_jclock_2.minutes_away(my_jclock_3.datetime))
    assert not my_jclock_2.is_minutes_away(my_jclock_3.datetime, 14.0)
    assert my_jclock_2.is_minutes_away(my_jclock_3.datetime, 14.00000001)
    assert my_jclock_2.is_minutes_away(my_jclock_3.datetime, 15.0)
    assert my_jclock_2.is_minutes_away(my_jclock_3.datetime, 15.99999999)
    assert not my_jclock_2.is_minutes_away(my_jclock_3.datetime, 16.0)

    # hey it works even across leap years
    print(my_jclock.minutes_away(my_jclock_2.datetime))
    print(my_jclock_2.minutes_away(my_jclock_4.datetime))



