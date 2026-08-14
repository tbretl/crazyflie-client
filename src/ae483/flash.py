"""Flash firmware to a Crazyflie."""

import argparse
import shlex
import subprocess
import sys
from datetime import datetime
from pathlib import Path


def default_firmware_dir():
    """crazyflie-firmware, assumed to be a sibling of crazyflie-client."""
    # .../ae483/crazyflie-client/src/ae483/flash.py -> .../ae483
    return Path(__file__).resolve().parents[3] / 'crazyflie-firmware'


def describe(path):
    """A one-line summary of a firmware binary, with its size and build time."""
    stat = path.stat()
    built = datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M')
    return f'{path.name} ({stat.st_size / 1e3:.0f} kB, built {built})'


def main():
    parser = argparse.ArgumentParser(
        prog='ae483-flash',
        description=(
            'Flash firmware to a Crazyflie over the radio. Assumes you have '
            'already built the firmware with "make" in crazyflie-firmware.'
        ),
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
    parser.add_argument(
        '--brushless',
        action='store_true',
        help='flash a brushless drone (cf21bl) instead of a brushed drone (cf2)',
    )
    parser.add_argument(
        '--firmware',
        type=Path,
        default=None,
        metavar='PATH',
        help=(
            'path to your crazyflie-firmware directory '
            '(default: a folder of that name next to crazyflie-client)'
        ),
    )
    args = parser.parse_args()

    platform = 'cf21bl' if args.brushless else 'cf2'
    kind = 'brushless' if args.brushless else 'brushed'

    firmware_dir = (
        args.firmware.expanduser().resolve()
        if args.firmware is not None
        else default_firmware_dir()
    )
    build_dir = firmware_dir / 'build'
    binary = build_dir / f'{platform}.bin'

    print()
    print(f'  drone     : {kind} ({platform})')
    print(f'  firmware  : {firmware_dir}')

    if not firmware_dir.is_dir():
        print(
            '\nThere is no directory at that path.\n\n'
            'By default we look for crazyflie-firmware next to crazyflie-client.\n'
            'If yours is somewhere else, say so with --firmware PATH.',
            file=sys.stderr,
        )
        return 1

    if not binary.is_file():
        found = sorted(build_dir.glob('*.bin')) if build_dir.is_dir() else []
        print(
            f'\nThere is no {platform}.bin in {build_dir}\n\n'
            f'Binaries found: {", ".join(p.name for p in found) or "(none)"}\n\n'
            'Did you run "make" in crazyflie-firmware? A brushless build also\n'
            'needs "make cf21bl_defconfig" first.',
            file=sys.stderr,
        )
        return 1

    print(f'  binary    : {describe(binary)}')
    for other in sorted(p for p in build_dir.glob('*.bin') if p != binary):
        print(f'  also here : {describe(other)}   <-- not being flashed')
    print(f'  uri       : {args.uri}')

    # The build directory records only the most recent configuration, and it is
    # normal to have built both platforms, so this is a staleness hint rather
    # than an error.
    auto_conf = build_dir / 'include' / 'config' / 'auto.conf'
    if auto_conf.is_file():
        configured = 'CONFIG_PLATFORM_CF21BL=y' in auto_conf.read_text()
        if configured != args.brushless:
            print(
                f'\n  NOTE: crazyflie-firmware was last configured for '
                f'{"brushless" if configured else "brushed"}, not {kind}.\n'
                f'        Check the build time above — {binary.name} may be out of date.'
            )

    # "stm32-fw" names the chip and the image type. It is the same for every
    # platform, so there is no reason to make students type it.
    arguments = ['flash', str(binary), 'stm32-fw', '-w', args.uri]
    cfloader = Path(sys.executable).parent / (
        'cfloader.exe' if sys.platform == 'win32' else 'cfloader'
    )

    if not cfloader.is_file():
        print(
            f'\nCould not find cfloader here:\n\n  {cfloader}\n\n'
            'It comes from the cfclient package. Try running "uv sync".',
            file=sys.stderr,
        )
        return 1

    print(f'\n  equivalent to: {shlex.join(["uv", "run", "cfloader"] + arguments)}\n')

    return subprocess.run([str(cfloader)] + arguments).returncode


if __name__ == '__main__':
    sys.exit(main())
