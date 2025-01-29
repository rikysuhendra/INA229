import os
import time
from datetime import datetime
import ina229

def move (y, x):
    print("\033[%d;%dH" % (y, x))
    
ina229 = ina229.INA229(busnum = 0, devnum = 0, shunt_ohms = 0.1)
ina229.configure()

# Clearing the Screen
os.system('clear')

try:
    while True:
        move (0, 0)
        print('VBUS voltage: ', ina229.get_vbus_voltage())
        print('Current: ', ina229.get_current())
        print('Power: ', ina229.get_power())
        print(datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
        time.sleep(0.1)
except KeyboardInterrupt:
    print('keyboard interrupted!')
