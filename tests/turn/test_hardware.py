from __future__ import annotations

import builtins
import math
from pathlib import Path
from types import ModuleType
import sys

import pytest

from pip_robot.turn import hardware
from pip_robot.turn.hardware import ImuReadError, Sh3001ImuAdapter


UTC = "2026-08-16T12:00:00+00:00"


class ScriptedSensor:
    def __init__(self, readings):
        self._readings = iter(readings)

    def _sh3001_getimudata(self):
        return next(self._readings)


class ConstantSensor:
    def __init__(
        self,
        reading=((8192.0, -8192.0, -8192.0), (2.0, 3.0, 4.0)),
    ):
        self._reading = reading

    def _sh3001_getimudata(self):
        return self._reading


class SteppingClock:
    def __init__(self, step):
        self._value = 0.0
        self._step = step

    def __call__(self):
        value = self._value
        self._value += self._step
        return value


class ScriptedClock:
    def __init__(self, values):
        self._values = iter(values)

    def __call__(self):
        return next(self._values)


def adapter_for(sensor, *, monotonic=lambda: 12.5):
    return Sh3001ImuAdapter(
        sensor=sensor,
        monotonic=monotonic,
        utc_now=lambda: UTC,
    )


def test_read_sample_uses_direct_injected_sensor_and_historical_orientation():
    adapter = adapter_for(ConstantSensor())

    sample = adapter.read_sample()

    assert sample.accel_xyz == (8192.0, -8192.0, -8192.0)
    assert sample.gyro_xyz == (2.0, 3.0, 4.0)
    assert sample.roll_deg == pytest.approx(35.26438968)
    assert sample.pitch_deg == pytest.approx(35.26438968)
    assert sample.monotonic_s == 12.5
    assert sample.utc_timestamp == UTC
    assert sample.valid is True


def test_read_sample_averages_every_read_before_deriving_attitude():
    sensor = ScriptedSensor(
        [
            ((3000, -6000, -9000), (10, 20, 30)),
            ((9000, -12000, -15000), (30, 40, 50)),
        ]
    )

    sample = adapter_for(sensor).read_sample(batch_size=2)

    assert sample.accel_xyz == (6000.0, -9000.0, -12000.0)
    assert sample.gyro_xyz == (20.0, 30.0, 40.0)
    assert sample.roll_deg == pytest.approx(47.96888623)
    assert sample.pitch_deg == pytest.approx(33.85451481)


@pytest.mark.parametrize(
    "reading",
    [
        None,
        (),
        ((16384, 0, 0),),
        ((16384, 0), (3, 4, 5)),
        ((16384, 0, 0), (4, 5, 6, 7)),
        ((16384, math.inf, 0), (4, 5, 6)),
        ((16384, 0, 0), (4, math.nan, 6)),
        ((True, 0, 0), (4, 5, 6)),
    ],
)
def test_read_sample_rejects_malformed_or_non_finite_sensor_data(reading):
    adapter = adapter_for(ScriptedSensor([reading]))

    with pytest.raises(ImuReadError):
        adapter.read_sample()


def test_read_sample_wraps_sensor_failures_as_typed_errors():
    class FailingSensor:
        def _sh3001_getimudata(self):
            raise OSError("i2c unavailable")

    with pytest.raises(ImuReadError, match="SH3001 read failed") as captured:
        adapter_for(FailingSensor()).read_sample()

    assert isinstance(captured.value.__cause__, OSError)


@pytest.mark.parametrize("batch_size", [0, -1, 1.5, True])
def test_read_sample_rejects_an_empty_or_invalid_batch(batch_size):
    with pytest.raises(ImuReadError):
        adapter_for(ConstantSensor()).read_sample(batch_size=batch_size)


def test_injected_sensor_factory_is_lazy_and_sensor_is_reused():
    constructed = []

    def factory():
        sensor = ConstantSensor()
        constructed.append(sensor)
        return sensor

    adapter = Sh3001ImuAdapter(
        sensor_factory=factory,
        monotonic=lambda: 1.0,
        utc_now=lambda: UTC,
    )
    assert constructed == []

    first = adapter.read_sample()
    second = adapter.read_sample()

    assert first.accel_xyz == second.accel_xyz == (8192.0, -8192.0, -8192.0)
    assert len(constructed) == 1


@pytest.mark.parametrize(
    "acceleration",
    [
        (0.0, 0.0, 0.0),
        (8191.0, 0.0, 0.0),
        (24577.0, 0.0, 0.0),
    ],
)
def test_read_sample_rejects_implausible_raw_gravity_vectors(acceleration):
    with pytest.raises(ImuReadError, match="gravity magnitude"):
        adapter_for(ConstantSensor((acceleration, (0.0, 0.0, 0.0)))).read_sample()


def test_read_sample_accepts_a_historical_normal_gravity_vector():
    sample = adapter_for(
        ConstantSensor(((-16318.0, -1040.0, 276.0), (0.0, 0.0, 0.0)))
    ).read_sample()

    assert sample.accel_xyz == (-16318.0, -1040.0, 276.0)


def test_read_sample_revalidates_the_averaged_gravity_vector():
    sensor = ScriptedSensor(
        [
            ((16384.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ((-16384.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ]
    )

    with pytest.raises(ImuReadError, match="averaged accelerometer"):
        adapter_for(sensor).read_sample(batch_size=2)


def test_read_sample_rejects_a_non_finite_average_from_finite_raw_axes():
    sensor = ConstantSensor(
        ((16384.0, 0.0, 0.0), (1e308, 1e308, 1e308))
    )

    with pytest.raises(ImuReadError, match="averaged gyroscope"):
        adapter_for(sensor).read_sample(batch_size=2)


def test_read_sample_rejects_a_non_finite_derived_attitude(monkeypatch):
    monkeypatch.setattr(hardware.math, "degrees", lambda _angle: math.inf)

    with pytest.raises(ImuReadError, match="derived attitude"):
        adapter_for(ConstantSensor()).read_sample()


def test_default_vendor_import_is_deferred_until_read(monkeypatch):
    imported = []
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "pidog.sh3001":
            imported.append(name)
            raise ModuleNotFoundError("vendor library deliberately blocked")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    adapter = Sh3001ImuAdapter(monotonic=lambda: 1.0, utc_now=lambda: UTC)
    assert imported == []

    with pytest.raises(ImuReadError, match="construct SH3001"):
        adapter.read_sample()

    assert imported == ["pidog.sh3001"]


def test_default_factory_passes_an_explicit_absolute_historical_config_path(monkeypatch):
    constructor_arguments = []

    class FakeSh3001:
        def __init__(self, **kwargs):
            constructor_arguments.append(kwargs)

    pidog = ModuleType("pidog")
    pidog.__path__ = []
    sh3001 = ModuleType("pidog.sh3001")
    sh3001.Sh3001 = FakeSh3001
    pidog.sh3001 = sh3001
    monkeypatch.setitem(sys.modules, "pidog", pidog)
    monkeypatch.setitem(sys.modules, "pidog.sh3001", sh3001)
    monkeypatch.setattr(
        hardware,
        "_install_robot_hat_i2c_scan_compatibility",
        lambda: False,
    )

    hardware._default_sensor_factory()

    expected = Path.home() / "pumpkin-pidog-agent" / "sh3001.config"
    assert constructor_arguments == [{"db": str(expected)}]
    assert expected.is_absolute()


def test_optional_robot_hat_compatibility_scans_with_injected_vendor_modules(monkeypatch):
    opened_buses = []

    class FakeBus:
        def __init__(self, bus_number):
            opened_buses.append(bus_number)

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            return False

        def write_quick(self, address):
            if address != 0x14:
                raise OSError("no device")

    class FakeI2C:
        def __init__(self):
            self._bus = 7

    robot_hat = ModuleType("robot_hat")
    robot_hat.__path__ = []
    robot_hat_i2c = ModuleType("robot_hat.i2c")
    robot_hat_i2c.I2C = FakeI2C
    robot_hat.i2c = robot_hat_i2c
    smbus2 = ModuleType("smbus2")
    smbus2.SMBus = FakeBus
    monkeypatch.setitem(sys.modules, "robot_hat", robot_hat)
    monkeypatch.setitem(sys.modules, "robot_hat.i2c", robot_hat_i2c)
    monkeypatch.setitem(sys.modules, "smbus2", smbus2)

    assert hardware._install_robot_hat_i2c_scan_compatibility() is True
    assert FakeI2C().scan() == [0x14]
    assert opened_buses == [7]


def test_two_second_freshness_diagnostic_accepts_enough_advancing_samples():
    adapter = adapter_for(ConstantSensor(), monotonic=SteppingClock(0.05))

    report = adapter.diagnose_freshness(duration_s=2.0, minimum_samples=20)

    assert report.sample_count == 20
    assert report.duration_s == pytest.approx(2.0)
    assert report.estimated_hz == pytest.approx(10.0)
    assert report.first_monotonic_s == pytest.approx(0.05)
    assert report.last_monotonic_s == pytest.approx(1.95)
    assert report.accepted is True


def test_freshness_diagnostic_rejects_when_minimum_count_is_not_met():
    adapter = adapter_for(ConstantSensor(), monotonic=SteppingClock(0.05))

    report = adapter.diagnose_freshness(duration_s=2.0, minimum_samples=21)

    assert report.sample_count == 20
    assert report.accepted is False


def test_freshness_diagnostic_rejects_a_single_non_advancing_timestamp():
    adapter = adapter_for(
        ConstantSensor(),
        monotonic=ScriptedClock([0.0, 0.1, 0.1]),
    )

    report = adapter.diagnose_freshness(duration_s=0.1, minimum_samples=1)

    assert report.sample_count == 1
    assert report.duration_s == pytest.approx(0.1)
    assert report.estimated_hz == 0.0
    assert report.first_monotonic_s == report.last_monotonic_s == pytest.approx(0.1)
    assert report.accepted is False


def test_freshness_diagnostic_rejects_an_interior_duplicate_timestamp():
    adapter = adapter_for(
        ConstantSensor(),
        monotonic=ScriptedClock([0.0, 0.1, 0.1, 0.1, 0.4, 0.5, 0.6]),
    )

    report = adapter.diagnose_freshness(duration_s=0.6, minimum_samples=3)

    assert report.sample_count == 3
    assert report.first_monotonic_s == pytest.approx(0.1)
    assert report.last_monotonic_s == pytest.approx(0.5)
    assert report.accepted is False


def test_freshness_diagnostic_rejects_an_interior_timestamp_regression():
    adapter = adapter_for(
        ConstantSensor(),
        monotonic=ScriptedClock([0.0, 0.2, 0.2, 0.1, 0.4, 0.5, 0.6]),
    )

    report = adapter.diagnose_freshness(duration_s=0.6, minimum_samples=3)

    assert report.sample_count == 3
    assert report.first_monotonic_s == pytest.approx(0.2)
    assert report.last_monotonic_s == pytest.approx(0.5)
    assert report.accepted is False


@pytest.mark.parametrize(
    ("duration_s", "minimum_samples"),
    [
        (0, 1),
        (-1, 1),
        (math.inf, 1),
        (True, 1),
        (1, 0),
        (1, -1),
        (1, 1.5),
        (1, True),
    ],
)
def test_freshness_diagnostic_rejects_invalid_limits(duration_s, minimum_samples):
    with pytest.raises(ImuReadError):
        adapter_for(ConstantSensor()).diagnose_freshness(
            duration_s=duration_s,
            minimum_samples=minimum_samples,
        )
