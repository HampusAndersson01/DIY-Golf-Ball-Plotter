from scripts.serial_port_diagnostic import classify_windows_serial_error, extract_windows_error_code


def test_extracts_wrapped_win32_error_from_pyserial_configuration_failure():
    error = Exception(
        "Cannot configure port, something went wrong. Original message: "
        "PermissionError(13, 'device not functioning', None, 31)"
    )

    assert extract_windows_error_code(error) == 31
    assert classify_windows_serial_error(error) == "driver_or_device_configuration_failure"


def test_classifies_port_in_use_and_missing_device_separately():
    assert classify_windows_serial_error(OSError(32, "sharing violation")) == "port_already_in_use"
    assert classify_windows_serial_error(OSError(2, "file not found")) == "device_not_found"
