from math import *
from pint import UnitRegistry
ureg = UnitRegistry()

volt = ureg.volt
farad = ureg.farad
microfarad = ureg.microfarad
sec = ureg.second
millisecond = ureg.millisecond
ohm = ureg.ohm
amp = ureg.amp

#http://pint.readthedocs.org/en/0.6/contexts.html?highlight=conversion
#time = resistance * capacitance

# c = pint.Context('ab')
# c.add_transformation('ohm', 'farad]',
# ...                      lambda ureg, x: ureg.speed_of_light / x)
# >>> c.add_transformation('[time]', '[length]',
# ...                      lambda ureg, x: ureg.speed_of_light * x)
# ureg.add_context(c)

sec_to_ohm_farads = sec/(ohm*farad)

#V = V0*e^(-t/(R*C))
# 
# where:
#     t = time that capacitor discharges
#     V = remaining voltage across capacitor leads after time t
#     R = resistence in Ohms of load across capacitor leads causing the discharge
#     C = capacitance in Farads of the capacitor

def has_units(v):
    try:
        v.units
        return True
    except AttributeError:
        return False

class Capacitor(object):
    
    def __init__(self, **kwargs):
        self._V0 = kwargs.pop('V0', 0)
        self._V = kwargs.pop('V', 0)
        self._R = kwargs.pop('R', 0)
        self._C = kwargs.pop('C', 0)
        self._t = kwargs.pop('t', 0)
        
    def calculate_t(self, **kwargs):
        # t = -R*C*ln(V/V0)
        
        V0 = kwargs.pop('V0', self._V0)
        if not has_units(V0):
            V0 = V0*volt
            
        V = kwargs.pop('V', self._V)
        if not has_units(V):
            V = V*volt
            
        R = kwargs.pop('R', self._R)
        if not has_units(R):
            R = R*ohm
            
        C = kwargs.pop('C', self._C)
        if not has_units(C):
            C = C*farad
        
        t = -R * C * log(V/V0)
        
        t *= sec_to_ohm_farads
        
        return t
        
if __name__ == '__main__':
    
    # At 5V the RPi consumes about 0.7A => R = V/I
    R = ((5*volt)/(0.7*amp)).to(ohm)
#     print R
    
    C = 4700*microfarad
    #C = 1*farad
    
    # Given a 4700uF capacitor charged to 5V, how long will it take a 7 Ohm load to discharge it to 4.7V?
    cap = Capacitor(C=C)
    t = cap.calculate_t(R=R, V0=5.*volt, V=4.7*volt)
    t.to(sec)
    print 't:', t.to(millisecond) 
    
    