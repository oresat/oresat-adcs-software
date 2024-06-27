import numpy as np

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

    def is_after(self, date_time):
        '''
        Checks to see if it is after the input time.

        Parameters
        ----------
        date_time : list
            List of integers, in following order:
            Year, Month, Day, Hour, Minute, Second

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
