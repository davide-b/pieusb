'''pieusb-udev: let ordinary users open the scanner on Linux.

    sudo pieusb-udev install      write the rules and apply them to plugged-in scanners
    sudo pieusb-udev uninstall    remove them
    pieusb-udev print             show the rules without installing

By default the rules tag every known scanner `uaccess`, which gives the user
logged in at the seat access, as for any desktop device. `--group NAME` grants
a group instead (MODE 0660), for headless machines and SSH sessions where no
one holds the seat.
'''

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

from pieusb.inquiry import KNOWN_PIDS, PIE_VENDOR_ID

RULES_PATH = Path('/etc/udev/rules.d/60-pieusb.rules')


def rules(group: str | None = None) -> str:
    '''The rules file's text: one line per known product id.'''
    access = f'GROUP="{group}", MODE="0660"' if group else 'TAG+="uaccess"'
    lines = ['# Reflecta / Pacific Image film scanners, for pieusb. Written by pieusb-udev.']
    lines += [f'SUBSYSTEM=="usb", ATTR{{idVendor}}=="{PIE_VENDOR_ID:04x}", '
              f'ATTR{{idProduct}}=="{pid:04x}", {access}' for pid in KNOWN_PIDS]
    return '\n'.join(lines) + '\n'


def reload_udev(run=subprocess.run) -> None:
    '''Reload the rules and re-apply them to scanners already plugged in.'''
    if shutil.which('udevadm') is None:
        print("udevadm not found: replug the scanner or reboot for the rules to apply")
        return
    run(['udevadm', 'control', '--reload-rules'], check=True)
    run(['udevadm', 'trigger', '--subsystem-match=usb',
         f'--attr-match=idVendor={PIE_VENDOR_ID:04x}'], check=True)


def install(path: Path = RULES_PATH, group: str | None = None, run=subprocess.run) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(rules(group))
    path.chmod(0o644)
    print(f"wrote {path}")
    reload_udev(run)
    print("done: a scanner already plugged in may need replugging")


def uninstall(path: Path = RULES_PATH, run=subprocess.run) -> None:
    if not path.exists():
        print(f"{path} is not there; nothing to remove")
        return
    path.unlink()
    print(f"removed {path}")
    reload_udev(run)


def sudo_hint(argv: list[str]) -> str:
    '''The command to rerun as root. The absolute path, because sudo's PATH
    usually leaves out a virtualenv's or pipx's bin directory.'''
    if sys.argv[0].endswith('.py'):
        return ' '.join(['sudo', sys.executable, '-m', 'pieusb.udev', *argv])
    exe = shutil.which(sys.argv[0]) or os.path.abspath(sys.argv[0])
    return ' '.join(['sudo', exe, *argv])


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    p = argparse.ArgumentParser(prog='pieusb-udev', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('action', choices=('install', 'uninstall', 'print'))
    p.add_argument('--group', metavar='NAME',
                   help='grant this group access instead of the seat user (uaccess)')
    args = p.parse_args(argv)

    if args.action == 'print':
        print(rules(args.group), end='')
        return 0
    if not sys.platform.startswith('linux'):
        print("udev rules are for Linux only")
        return 1
    if os.geteuid() != 0:
        print(f"this needs root:\n\n    {sudo_hint(argv)}\n")
        return 1
    if args.action == 'install':
        install(group=args.group)
    else:
        uninstall()
    return 0


if __name__ == '__main__':
    sys.exit(main())
