"""Check that resting diagnostics cannot send torque or movement commands."""

import contextlib
import csv
import io
from pathlib import Path
import runpy
import tempfile
import unittest
from unittest.mock import Mock, patch


class RestCheckTests(unittest.TestCase):
    def load_script(self):
        # Only query methods and serial cleanup exist on this restricted mock.
        # Loading through a fake pymycobot module needs no installed robot SDK.
        self.client = Mock(spec=["get_system_version", "is_free_mode",
                                 "is_servo_enable", "get_servo_status",
                                 "get_servo_voltages", "get_servo_temps",
                                 "get_angles", "is_moving",
                                 "get_error_information", "_serial_port"])
        self.client.get_angles.return_value = [43.33, -14.5, 11.16, 120.67, 82.88, -45.7]
        self.client.is_moving.return_value = 0
        self.client.get_error_information.return_value = 0
        self.sdk = Mock(spec=["MyCobot280", "PI_PORT", "PI_BAUD", "__version__"])
        self.sdk.PI_PORT = "/dev/ttyAMA0"
        self.sdk.PI_BAUD = 1000000
        self.sdk.__version__ = "4.0.6"
        self.sdk.MyCobot280.return_value = self.client
        path = Path(__file__).resolve().parents[1] / "pi_rest_check.py"

        # Import the script without entering its direct-execution main block.
        with patch.dict("sys.modules", {"pymycobot": self.sdk}):
            namespace = runpy.run_path(str(path))

        self.sdk.MyCobot280.assert_not_called()
        return namespace["main"]

    def test_records_30_samples_without_motion_methods(self):
        # Redirect CSV output into a temporary directory and skip actual sleeps.
        # Check the saved header plus 30 rows, not just the printed summary.
        main = self.load_script()
        with tempfile.TemporaryDirectory() as folder:
            # Capture builtin open before patching it to avoid recursion.
            real_open = open
            def redirected_open(filename, *args, **kwargs):
                return real_open(Path(folder) / filename, *args, **kwargs)

            output = io.StringIO()
            with patch("builtins.open", side_effect=redirected_open), patch("time.sleep"):
                with contextlib.redirect_stdout(output):
                    main()

            logs = list(Path(folder).glob("pi_rest_*.csv"))

            self.assertEqual(len(logs), 1)
            with logs[0].open(newline="") as log:
                rows = list(csv.reader(log))

            self.assertEqual(len(rows), 31)
            self.assertEqual(rows[0][1:7], ["J1_deg", "J2_deg", "J3_deg",
                                          "J4_deg", "J5_deg", "J6_deg"])

        self.assertIn("[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]", output.getvalue())
        self.client._serial_port.close.assert_called_once()

    def test_unexpected_moving_state_stops_queries_and_closes(self):
        # The first moving sample should remain saved, but end further sampling.
        main = self.load_script()
        self.client.is_moving.return_value = 1
        real_open = open
        with tempfile.TemporaryDirectory() as folder:
            def redirected_open(filename, *args, **kwargs):
                return real_open(Path(folder) / filename, *args, **kwargs)

            with patch("builtins.open", side_effect=redirected_open):
                with contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(RuntimeError, "Unexpected motion"):
                        main()

            with next(Path(folder).glob("pi_rest_*.csv")).open(newline="") as log:
                self.assertEqual(len(list(csv.reader(log))), 2)

        self.client.get_angles.assert_called_once()
        self.client._serial_port.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
