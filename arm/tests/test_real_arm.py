"""Exercise hardware behavior with SDK doubles; never open a real port."""

import contextlib
from importlib import metadata
import io
import json
import unittest
from unittest.mock import Mock, patch

from bucke_arm.arm import RealArm
from bucke_arm.__main__ import main


class HardwareTests(unittest.TestCase):
    def client(self):
        # A restricted mock raises if adapter code tries a power/motion method.
        client = Mock(spec=["get_angles", "get_coords", "_serial_port"])
        client.get_angles.return_value = [1.0] * 6
        client.get_coords.return_value = [2.0] * 6
        return client

    def test_constructor_uses_tested_settings(self):
        # Replace the imported SDK and installed-version lookup before creating
        # the adapter, so construction cannot open a physical serial port.
        sdk = Mock(spec=["MyCobot280", "PI_PORT", "PI_BAUD"])
        sdk.PI_PORT = "/dev/ttyAMA0"
        sdk.PI_BAUD = 1000000
        sdk.MyCobot280.return_value = self.client()
        with patch("bucke_arm.arm.metadata.version", return_value="4.0.6"):
            with patch.dict("sys.modules", {"pymycobot": sdk}):
                arm = RealArm()
                arm.close()

        sdk.MyCobot280.assert_called_once_with("/dev/ttyAMA0", 1000000)

    def test_changed_pi_constants_never_open_connection(self):
        # An unexpected device path must be rejected before SDK construction.
        sdk = Mock(spec=["MyCobot280", "PI_PORT", "PI_BAUD"])
        sdk.PI_PORT = "/dev/other"
        sdk.PI_BAUD = 1000000
        with patch("bucke_arm.arm.metadata.version", return_value="4.0.6"):
            with patch.dict("sys.modules", {"pymycobot": sdk}):
                with self.assertRaisesRegex(RuntimeError, "constants differ"):
                    RealArm()

        sdk.MyCobot280.assert_not_called()

    def test_unverified_sdk_never_constructs_client(self):
        # Unknown SDK versions must fail without trying a connection anyway.
        sdk = Mock(spec=["MyCobot280"])
        with patch("bucke_arm.arm.metadata.version", return_value="unknown"):
            with patch.dict("sys.modules", {"pymycobot": sdk}):
                with self.assertRaisesRegex(RuntimeError, "Expected verified"):
                    RealArm()

        sdk.MyCobot280.assert_not_called()

    def test_missing_sdk_has_useful_error(self):
        # Translate package-discovery failure into the adapter's readable error.
        with patch("bucke_arm.arm.metadata.version", side_effect=metadata.PackageNotFoundError):
            with self.assertRaisesRegex(RuntimeError, "missing"):
                RealArm()

    def test_cli_reports_hardware_and_closes_connection(self):
        # Exercise the hardware CLI branch with a fake client, checking source
        # labeling, permitted reads, and connection cleanup together.
        client = self.client()
        output = io.StringIO()
        with patch("bucke_arm.__main__.RealArm", return_value=RealArm(client)):
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["status", "--backend", "hardware"]), 0)

        self.assertEqual(json.loads(output.getvalue())["source"], "hardware")
        client.get_angles.assert_called_once_with()
        client.get_coords.assert_called_once_with()
        client._serial_port.close.assert_called_once_with()

    def test_failed_reads_close_without_fake_fallback(self):
        # Simulate missing, malformed, and failed serial replies. None may be
        # replaced with successful-looking simulated readings after failure.
        for failure in (None, [float("nan")] * 6, [1.0] * 5, OSError("serial read failed")):
            with self.subTest(failure=failure):
                client = self.client()
                if isinstance(failure, Exception):
                    client.get_angles.side_effect = failure
                else:
                    client.get_angles.return_value = failure

                output, error = io.StringIO(), io.StringIO()
                with patch("bucke_arm.__main__.RealArm", return_value=RealArm(client)):
                    with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
                        self.assertEqual(main(["status", "--backend", "hardware"]), 1)

                self.assertEqual(output.getvalue(), "")
                self.assertIn("Status failed:", error.getvalue())
                client._serial_port.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
