import numpy as np
from datetime import datetime, timedelta

class Clock():
    '''Calculations from Fundamentals of Spacecraft Attitude Determination and Control
    by Markely and Crassidis. Please pardon the magic numbers.
    Parameters are for the Gregorian calendar date and time.

    Parameters
    ----------
    year : int
    month : int
    day : int
    hour : int
    minute : int
    second : float
    '''
    def __init__(self, year, month, day, hour, minute, second):
        self.absolute = 0
        self.year = year
        self.month = month
        self.day = day
        self.hour = hour
        self.minute = minute
        self.second = second
        self.leap_year = self.is_leap_year()
        self.leap_second = False

    def is_after(self, date_time, other_datetime=None):
        '''
        Checks to see if it is after the input time.

        Parameters
        ----------
        date_time : list
            List of integers, in following order:
            Year, Month, Day, Hour, Minute, Second
        other_datetime : datetime object

        Returns
        -------
        bool
            True if it is after the input time.
        '''
        
        if self.year < date_time[0]: return False
        if self.month < date_time[1]: return False
        if self.day < date_time[2]: return False
        if self.hour < date_time[3]: return False
        if self.minute < date_time[4]: return False
        if self.second < date_time[5]: return False
        return True

    def is_minutes_away(self, date_time, minutes):
        '''This checks to see if the provided time is in the next whenever minutes.
        For example, if the time is between 14 and 15 minutes away, this function
        will only return True if 14 or 15 is passed 

        Parameters
        ----------
        date_time : list
            List of integers, in following order:
            Year, Month, Day, Hour, Minute, Second
        minutes : int
            Length of window. Must be less than 60 for defined behavior.

        Returns
        -------
        int
            True if the time is within the next whenever minutes.
        '''
        if self.is_after(date_time): return False
        setup_time = date_time.copy()
        if date_time[4] < minutes:
            if date_time[3] == 0:
                if date_time[2] == 1:
                    if date_time[1] == 1:
                        setup_time[0] -= 1
                        setup_time[1] = 12
                    else:
                        setup_time[1] -= 1
                    setup_time[2] = self.days_in_month(setup_time[1], self.leap_year)
                else:
                    setup_time[2] -= 1
                setup_time[3] = 23
            else:
                setup_time[3] -= 1
            setup_time[4] += 60 - minutes
        else:
            setup_time[4] -= minutes
        return self.is_after(setup_time)

    def schedule_leap_second(self, year, month, day, hour, minute, second):
        '''Not implemented. Might eventually need method to set leap seconds.

        Parameters
        ----------
        year : int
        month : int
        day : int
        hour : int
        minute : int
        second : float
        '''
        pass # figure it out later

    def is_leap_second(self):
        '''Not implemented. Might eventually need method to check leap seconds.
        '''
        pass # probably just check whether state matches schedule

    def is_leap_year(self):
        '''
        Check to see whether it is a leap year.

        Returns
        -------
        bool
            True if it is a leap year, and False otherwise.
        '''
        if not (self.year % 4 == 0):
            return False
        elif not (self.year % 100 == 0):
            return True
        elif not (self.year % 400 == 0):
            return False
        else:
            return True

    def days_in_month(self, month, leap_year):
        '''This returns the number of days in the given month.

        Parameters
        ----------
        month : int
            Number of month.
        leap_year : bool
            True if it's a leap year.

        Returns
        -------
        int
            Number of days in month.
        '''
        if month in [1, 3, 5, 7, 8, 10, 12]: return 31
        if month in [4, 6, 9, 11]: return 30
        if leap_year: return 29
        return 28

    def tick(self, dt):
        '''This method is for advancing the clock by one time step, it is a somewhat naive implementation.

        Parameters
        ----------
        dt : float
            The length of time to move forward by.
        '''
        self.second += dt
        self.absolute += dt
        if ((not self.leap_second and abs(self.second - 60) < 0.00001) or
            (self.leap_second and abs(self.second - 61) < 0.00001)):
            self.second = 0
            self.minute += 1

        if self.minute == 60:
            self.minute = 0
            self.hour += 1
        if self.hour == 24:
            self.hour = 0
            self.day += 1

        if self.day > self.days_in_month(self.month, self.leap_year):
            self.day = 1
            self.month += 1

        if self.month == 13:
            self.month = 1
            self.year += 1
            self.leap_year = self.is_leap_year()

    def julian_date(self, hour, minute, second):
        '''This method calculates the Julian date.

        Parameters
        ----------
        hour : int
        minute : int
        second : float

        Returns
        -------
        float
            The Julian date.
        '''
        term1 = int(7/4 * (self.year + int((self.month + 9) / 12)))
        term2 = int(275*self.month / 9)
        term3 = (60*hour + minute + second/(60+self.leap_second)) / 1440
        return 1721013.5 + 367*self.year + self.day - term1 + term2 + term3

    def centuries_elapsed(self):
        '''This calulates how many centuries have elapsed from epoch J2000.0 to zero hour now

        Returns
        -------
        float
            Centuries elapsed since epoch.
        '''
        return (self.julian_date(0,0,0) - 2451545) / 36525


    def gmst_seconds(self):
        '''Greenwich Mean Sidereal time in seconds

        Returns
        -------
        float
            Greenwich Mean Sidereal time in seconds.
        '''
        centuries = self.centuries_elapsed()
        term1 = 8640184.812866 * centuries + 0.093104 * centuries**2 - 6.2*10**(-6) * centuries**3
        term2 = 1.002737909350795 * (3600 * self.hour + 60 * self.minute + self.second)
        return 24110.54841 + term1 + term2

    def theta_gmst(self):
        '''This angle is the angle between vernal equinox and Greenwich mean line.

        Returns
        -------
        float
            Angle offset (rad) between the ECEF and ECI frames of reference.
        '''
        gmst = self.gmst_seconds() % 86400
        return np.radians(gmst / 240)



class JClock:
    '''Calculations from Fundamentals of Spacecraft Attitude Determination and Control
    by Markely and Crassidis. Please pardon the magic numbers.
    Parameters are for the Gregorian calendar date and time.
    Parameters
    ----------
    year : int
    month : int
    day : int
    hour : int
    minute : int
    second : int

    Remember that datetime is immutable!
    '''
    def __init__(self, year, month, day, hour, minute, second):
        self.set_datetime(year, month, day, hour, minute, second)
        self.absolute = 0
        self.ref_datetime = datetime(2000, 1, 1, 12, 0, 0)

        # is_leap_year is probably only used for ticking the clock forward
        # self.leap_year = self.is_leap_year()
    
    
    def __eq__(self, other):
        return self.datetime == other.datetime
    
    def tick(self) -> None:
        '''Increase the datetime by some one second'''
        # simply add a timedelta object to the datetime object
        self.datetime = self.datetime + timedelta(seconds=1)

    def sync(self) -> float:
        '''Sets the datetime to datetime.now() and returns the difference in seconds'''
        old_datetime = self.datetime
        self.datetime = datetime.now()
        return (self.datetime - old_datetime).total_seconds()

    def set_datetime(self, year, month, day, hour, minute, second) -> None:
        '''Sets the date time, good for synchronization'''
        self.datetime = datetime(year, month, day, hour, minute, second)

    def is_after(self, other_datetime) -> bool:
        '''Checks if this datetime is after the given datetime'''
        return self.datetime > other_datetime

    def minutes_away(self, other_datetime) -> float:
        '''Returns the number of minutes away as a float'''
        return float((other_datetime - self.datetime).total_seconds() / 60)

    def is_minutes_away(self, other_datetime, minutes) -> bool:
        '''Checks if another datetime is within 1 minute of a specified number of minues away'''
        return abs(float((other_datetime - self.datetime).total_seconds() / 60) - minutes) < 1.0


    def julian_date(self) -> float:
        '''This method calculates the Julian date'''
        # use the number of seconds to get the float
        return ((self.datetime - self.ref_datetime).total_seconds())/86400 + 2451545.0

    def centuries_elapsed(self) -> float:
        '''Determines julian centuries passed (calculated based on midnight of the day'''
        julian_seconds = (self.datetime - self.ref_datetime).total_seconds() 
        return (julian_seconds - 3600*self.datetime.hour - 60*self.datetime.minute - self.datetime.second)/(86400*36525)
    
    def gmst_seconds(self) -> float:
        '''Greenwich Mean Sidereal time in seconds'''
        centuries = self.centuries_elapsed()
        gmst_zero = 24110.54841 + 8640184.812866 * centuries + 0.093104 * centuries**2 - 6.2*10**(-6) * centuries**3
        earth_rotation = 1.002737909350795 * (3600 * (self.datetime.hour) + 60 * self.datetime.minute + self.datetime.second)
        return gmst_zero + earth_rotation

    def theta_gmst(self) -> float:
        '''This angle is the angle between vernal equinox and Greenwich mean line (in radians)'''
        return np.radians((self.gmst_seconds()%86400) / 240)
