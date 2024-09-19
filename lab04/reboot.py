import sys
from cflib.utils.power_switch import PowerSwitch

if len(sys.argv) != 2:
    print("Error: uri is missing")
    print('Usage: {} uri'.format(sys.argv[0]))
    sys.exit(-1)

PowerSwitch(sys.argv[1]).stm_power_cycle()