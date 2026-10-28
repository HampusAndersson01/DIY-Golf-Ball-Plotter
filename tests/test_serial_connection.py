from types import SimpleNamespace

import pytest
import serial

from app import create_app
from app.services import pipeline_core
from serial.tools import list_ports


@pytest.fixture
def app(monkeypatch):
    app = create_app()
    app.config.update(TESTING=True, SERIAL_PORT='')
    monkeypatch.setattr(pipeline_core, 'grbl', None)
    monkeypatch.setattr(pipeline_core.time, 'sleep', lambda _: None)
    return app


def port(device, vid=0x1A86):
    return SimpleNamespace(device=device, description='USB controller', hwid='USB', vid=vid)


@pytest.mark.parametrize('device', ['/dev/ttyUSB0', '/dev/ttyACM0', 'COM4', '/dev/cu.usbserial-1'])
def test_ports_endpoint_lists_native_devices_and_selects_single_usb(app, monkeypatch, device):
    monkeypatch.setattr(list_ports, 'comports', lambda: [port(device)])
    response = app.test_client().get('/serial-ports')
    assert response.status_code == 200
    data = response.get_json()
    assert data['ports'][0]['device'] == device
    assert data['selected_port'] == device


def test_connect_selected_port_reaches_serial_driver(app, monkeypatch):
    opened = []
    fake = SimpleNamespace(is_open=True, in_waiting=0, write=lambda _: None)
    def open_serial(device, baud, **kwargs):
        opened.append((device, baud))
        return fake
    monkeypatch.setattr(pipeline_core.serial, 'Serial', open_serial)
    # Bypass unrelated motor policy commands after the connection handshake.
    monkeypatch.setattr(app.extensions['machine_service'], 'apply_stepper_hold_policy', lambda *a, **kw: None)
    response = app.test_client().post('/connect', json={'port': '/dev/ttyUSB0'})
    assert response.status_code == 200
    assert opened == [('/dev/ttyUSB0', 115200)]
    assert app.config['SERIAL_PORT'] == '/dev/ttyUSB0'


def test_auto_connect_uses_the_only_usb_controller(app, monkeypatch):
    monkeypatch.setattr(list_ports, 'comports', lambda: [port('/dev/ttyS0', None), port('/dev/ttyUSB0')])
    opened = []
    def open_serial(device, baud, **kwargs):
        opened.append(device)
        return SimpleNamespace(is_open=True, in_waiting=0, write=lambda _: None)
    monkeypatch.setattr(pipeline_core.serial, 'Serial', open_serial)
    app.extensions['serial_service'].connect()
    assert opened == ['/dev/ttyUSB0']


@pytest.mark.parametrize('devices', [[], [port('/dev/ttyUSB0'), port('/dev/ttyUSB1')], [port('/dev/ttyS0', None)]])
def test_auto_connect_requires_one_usb_device(app, monkeypatch, devices):
    monkeypatch.setattr(list_ports, 'comports', lambda: devices)
    response = app.test_client().post('/connect', json={})
    assert response.status_code == 400
    assert 'serial port' in response.get_json()['error'].lower()


def test_permission_error_explains_linux_access(app, monkeypatch):
    monkeypatch.setattr('app.services.serial_service.sys.platform', 'linux')
    def denied(*a, **kw):
        raise serial.SerialException(13, 'Permission denied')
    monkeypatch.setattr(pipeline_core.serial, 'Serial', denied)
    response = app.test_client().post('/connect', json={'port': '/dev/ttyUSB0'})
    assert response.status_code == 500
    assert 'dialout' in response.get_json()['error']
    assert '/dev/ttyUSB0' in response.get_json()['error']


def test_connected_controller_cannot_be_switched(app, monkeypatch):
    app.config['SERIAL_PORT'] = '/dev/ttyUSB0'
    monkeypatch.setattr(pipeline_core, 'grbl', SimpleNamespace(is_open=True))
    response = app.test_client().post('/connect', json={'port': '/dev/ttyUSB1'})
    assert response.status_code == 400
    assert app.config['SERIAL_PORT'] == '/dev/ttyUSB0'


def test_stale_windows_setting_does_not_hide_linux_device(app, monkeypatch):
    app.config['SERIAL_PORT'] = 'COM12'
    monkeypatch.setattr(list_ports, 'comports', lambda: [port('/dev/ttyUSB0')])
    assert app.test_client().get('/serial-ports').get_json()['selected_port'] == '/dev/ttyUSB0'


def test_configured_stable_device_alias_is_selectable(app, monkeypatch, tmp_path):
    device = tmp_path / 'ttyUSB0'
    device.touch()
    alias = tmp_path / 'usb-controller'
    alias.symlink_to(device)
    app.config['SERIAL_PORT'] = str(alias)
    monkeypatch.setattr(list_ports, 'comports', lambda: [port(str(device))])
    data = app.test_client().get('/serial-ports').get_json()
    assert data['selected_port'] == str(alias)
    assert any(entry['device'] == str(alias) for entry in data['ports'])
