import argparse
from cflib.utils.power_switch import PowerSwitch


def main():
    parser = argparse.ArgumentParser(
        prog='ae483-reboot',
        description='Power-cycle a Crazyflie over the radio.',
    )
    parser.add_argument(
        'uri',
        help=(
            'uniform resource identifier of the drone, of the form '
            '"radio://dongle/channel/speed/address" '
            '(e.g., "radio://0/80/2M/E7E7E7E7E7"). '
            'The channel is likely the only value unique to your drone.'
        ),
    )
    args = parser.parse_args()
    PowerSwitch(args.uri).stm_power_cycle()


if __name__ == '__main__':
    main()