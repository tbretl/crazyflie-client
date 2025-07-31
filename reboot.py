import sys
from cflib.utils.power_switch import PowerSwitch

if len(sys.argv) != 2:
    print('=============================================================')
    print('ERROR\n')
    print('Call this script with exactly one argument as\n')
    print(f'python {sys.argv[0]} uri\n')
    print('where uri is a uniform resource identifier of the form\n')
    print('"radio://dongle/channel/speed/address"\n')
    print('in which\n')
    print('- dongle is the USB dongle number (e.g., 0)')
    print('- channel is the radio channel number (e.g., 80)')
    print('- speed is the radio speed (e.g., 2M)')
    print('- address is the radio address (e.g., E7E7E7E7E7).\n')
    print('The radio channel number is most likely the only value')
    print('that is unique to your drone.')
    print('=============================================================')
    sys.exit(-1)

PowerSwitch(sys.argv[1]).stm_power_cycle()