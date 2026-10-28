"""Verify the simple script reads only, and closes on diagnostic failure."""

import contextlib
import io
from pathlib import Path
import runpy
import unittest
from unittest.mock import Mock, patch


class PiStatusTests(unittest.TestCase):
    def load_script(self):
        # spec restricts the fake to query/cleanup methods; any attempt to use
        # an unlisted power or motion method raises AttributeError immediately.
        self.client = Mock(spec=["get_system_version", "get_angles", "is_moving",
                                 "get_error_information", "get_servo_status",
                                 "get_servo_voltages", "get_servo_temps",
                                 "_serial_port"])
        self.sdk = Mock(spec=["MyCobot280", "PI_PORT", "PI_BAUD", "__version__"])
        self.sdk.PI_PORT = "/dev/ttyAMA0"
        self.sdk.PI_BAUD = 1000000
        self.sdk.__version__ = "4.0.6"
        self.sdk.MyCobot280.return_value = self.client
        path = Path(__file__).resolve().parents[1] / "pi_status.py"

        # run_path loads definitions without executing the __main__ entry point.
        with patch.dict("sys.modules", {"pymycobot": self.sdk}):
            namespace = runpy.run_path(str(path))

        self.sdk.MyCobot280.assert_not_called()
        return namespace["main"]

    def test_reads_only_and_closes(self):
        # Remove real pauses while retaining all three diagnostic query rounds.
        main = self.load_script()
        with contextlib.redirect_stdout(io.StringIO()), patch("time.sleep"):
            main()

        self.sdk.MyCobot280.assert_called_once_with("/dev/ttyAMA0", 1000000)
        self.assertEqual(self.client.get_angles.call_count, 3)
        self.client.get_servo_status.assert_called_once()
        self.client._serial_port.close.assert_called_once()

    def test_failed_query_still_closes_without_another_action(self):
        # A failed first angle query should jump to cleanup, not later diagnostics.
        main = self.load_script()
        self.client.get_angles.side_effect = OSError("read failed")
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(OSError):
                main()

        self.client._serial_port.close.assert_called_once()
        self.client.get_servo_status.assert_not_called()

    def test_changed_connection_settings_are_rejected_before_connecting(self):
        # Change the loaded script's constant, not the real installed SDK settings.
        main = self.load_script()
        main.__globals__["PI_PORT"] = "/dev/other"
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):
                main()

        self.sdk.MyCobot280.assert_not_called()


if __name__ == "__main__":
    unittest.main()
