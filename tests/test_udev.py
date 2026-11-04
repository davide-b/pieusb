'''pieusb-udev, against a temporary directory. No root, no udevadm.'''

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pieusb import udev
from pieusb.inquiry import KNOWN_PIDS


class Rules(unittest.TestCase):

    def test_every_known_product_gets_a_line(self):
        text = udev.rules()
        for pid in KNOWN_PIDS:
            self.assertIn(f'ATTR{{idVendor}}=="05e3", ATTR{{idProduct}}=="{pid:04x}", '
                          f'TAG+="uaccess"', text)

    def test_the_readme_rule_is_what_it_writes(self):
        readme = (Path(__file__).resolve().parents[1] / 'README.md').read_text()
        for line in udev.rules().splitlines()[1:]:
            self.assertIn(line, readme)

    def test_a_group_replaces_uaccess(self):
        text = udev.rules('scanner')
        self.assertNotIn('uaccess', text)
        self.assertEqual(text.count('GROUP="scanner", MODE="0660"'), len(KNOWN_PIDS))


class Install(unittest.TestCase):

    def setUp(self):
        self.path = Path(self.enterContext(tempfile.TemporaryDirectory())) / 'rules.d' / '60-pieusb.rules'
        self.calls = []
        self.enterContext(mock.patch('builtins.print'))
        self.enterContext(mock.patch.object(udev.shutil, 'which', return_value='/usr/bin/udevadm'))

    def run_cmd(self, cmd, check):
        self.calls.append(cmd)

    def test_install_writes_the_rules_and_reloads_udev(self):
        udev.install(self.path, run=self.run_cmd)
        self.assertEqual(self.path.read_text(), udev.rules())
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o644)
        self.assertEqual(self.calls[0], ['udevadm', 'control', '--reload-rules'])
        self.assertIn('--attr-match=idVendor=05e3', self.calls[1])

    def test_uninstall_removes_them(self):
        udev.install(self.path, run=self.run_cmd)
        udev.uninstall(self.path, run=self.run_cmd)
        self.assertFalse(self.path.exists())

    def test_uninstall_without_rules_does_nothing(self):
        udev.uninstall(self.path, run=self.run_cmd)
        self.assertEqual(self.calls, [])


class Main(unittest.TestCase):

    @unittest.skipUnless(sys.platform.startswith('linux'), 'Linux only')
    def test_without_root_it_prints_the_sudo_command_by_absolute_path(self):
        with mock.patch.object(udev.os, 'geteuid', return_value=1000), \
                mock.patch.object(udev.sys, 'argv', ['pieusb-udev', 'install']), \
                mock.patch.object(udev.shutil, 'which', return_value='/home/u/.venv/bin/pieusb-udev'), \
                mock.patch('builtins.print') as printed, \
                mock.patch.object(udev, 'install') as install:
            self.assertEqual(udev.main(['install']), 1)
        install.assert_not_called()
        self.assertIn('sudo /home/u/.venv/bin/pieusb-udev install', printed.call_args[0][0])

    def test_print_needs_no_root(self):
        with mock.patch('builtins.print') as printed:
            self.assertEqual(udev.main(['print']), 0)
        self.assertEqual(printed.call_args[0][0], udev.rules())
