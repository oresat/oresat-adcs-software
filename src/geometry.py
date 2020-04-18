import numpy as np

# Reference Fundamentals of Spacecraft Attitude Determination and Control
# by Markely and Crassidis for understanding, pardon the magic numbers
# This is needed to translate between ECEF and ECI coordinates
class Clock():
    def __init__(year, month, day, hour, minute, second, dt):
        self.year = year
        self.month = month
        self.day = day
        self.hour = hour
        self.minute = minute
        self.second = second
        self.leap_year = is_leap_year()
        self.leap_second = False

    def schedule_leap_second(year, month, day, hour, minute, second):
        pass # figure it out later
    def is_leap_second(self):
        pass # probably just check whether state matches schedule
    def is_leap_year(self):
        if not (self.year % 4 == 0):
            return False
        elif not (self.year % 100 == 0):
            return True
        elif not (self.year % 400 == 0):
            return False
        else:
            return True
    # advance one time step
    def tick(self): # naive implementation, I hate the Gregorian calendar
        self.second += self.dt
        if ((not self.leap_second and self.second + self.dt == 60) or
            (self.leap_second and self.second == 61)):
            self.second = 0
            self.minute += 1

        if self.minute == 60:
            self.minute = 0
            self.hour += 1
        if self.hour == 24:
            self.hour = 0
            self.day += 1

        if ((self.month in [1, 3, 5, 7, 8, 10, 12] and self.day == 32) or
            (self.month in [4, 6, 9, 11] and self.day == 31) or
            (self.month == 2 and ((not self.leap_year and self.day == 29) or
                                    (self.leap_year and self.day == 30)))):
            self.day = 1
            self.month += 1

        if self.month == 13:
            self.month = 1
            self.year += 1
            self.leap_year = self.is_leap_year()
    # we could cache some of this intelligently
    def julian_date(self, hour, minute, second):
        term1 = int(7/4 * (self.year + int((self.month + 9) / 12)))
        term2 = int(275*self.month / 9)
        term3 = (60*hour + minute + second/(60+self.leap_second)) / 1440
        return 1721013.5 + 367*self.year + self.day - term1 + term2 + term3
    # from epoch J2000.0 to zero hour now
    def centuries_elapsed(self):
        return (self.julian_date(0,0,0) - 2451545) / 36525
    # Greenwich Mean Sidereal time
    def gmst_seconds(self):
        centuries = self.centuries_elapsed()
        term1 = 8640184.812866 * centuries + 0.093104 * centuries**2 - 6.2*10**(-6) * centuries**3
        term2 = 1.002737909350795 * (3600 * self.hour + 60 * self.minute + self.second)
        return 24110.54841 + term1 + term2
    # angle between vernal equinox and Greenwich mean line
    def theta_gmst(self):
        gmst = self.gmst_seconds() % 86400
        return np.radians(gmst / 240)
