"""Fresh, direct SH3001 acquisition without constructing a PiDog controller."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from numbers import Real
import time
from typing import Any

from .models import ImuSample


class ImuReadError(RuntimeError):
    """A live IMU sample could not be acquired or validated."""


@dataclass(frozen=True)
class ImuFreshnessReport:
    """Observed rate and timestamp progress during a bounded live diagnostic."""

    sample_count: int
    duration_s: float
    estimated_hz: float
    first_monotonic_s: float | None
    last_monotonic_s: float | None
    accepted: bool


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _native_i2c_scan(bus_number: int) -> list[int]:
    """Scan with smbus2 for Robot HAT versions that require i2cdetect."""
    from smbus2 import SMBus

    found: list[int] = []
    with SMBus(int(bus_number)) as bus:
        for address in range(0x03, 0x78):
            try:
                bus.write_quick(address)
            except OSError:
                continue
            found.append(address)
    return found


def _install_robot_hat_i2c_scan_compatibility() -> bool:
    """Install the historical scan workaround when Robot HAT is available."""
    try:
        import robot_hat.i2c as robot_hat_i2c
    except (ImportError, AttributeError):
        return False

    def scan(i2c: Any) -> list[int]:
        return _native_i2c_scan(getattr(i2c, "_bus", 1))

    robot_hat_i2c.I2C.scan = scan
    return True


def _default_sensor_factory() -> Any:
    _install_robot_hat_i2c_scan_compatibility()
    from pidog.sh3001 import Sh3001

    return Sh3001()


def _finite_triple(value: object, label: str) -> tuple[float, float, float]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
        or len(value) != 3
    ):
        raise ImuReadError(f"{label} must be a three-axis sequence")

    result: list[float] = []
    for component in value:
        if (
            isinstance(component, bool)
            or not isinstance(component, Real)
            or not math.isfinite(component)
        ):
            raise ImuReadError(f"{label} must contain finite numbers")
        result.append(float(component))
    return result[0], result[1], result[2]


def _finite_clock_value(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ImuReadError(f"{label} must return a finite number")
    return float(value)


class Sh3001ImuAdapter:
    """Read fresh accelerometer and gyroscope values directly from SH3001."""

    def __init__(
        self,
        sensor: Any = None,
        sensor_factory: Callable[[], Any] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        utc_now: Callable[[], str] = _utc_now,
    ) -> None:
        self._sensor = sensor
        self._sensor_factory = sensor_factory or _default_sensor_factory
        self._monotonic = monotonic
        self._utc_now = utc_now

    def _get_sensor(self) -> Any:
        if self._sensor is None:
            try:
                self._sensor = self._sensor_factory()
            except Exception as exc:
                raise ImuReadError("failed to construct SH3001 sensor") from exc
        return self._sensor

    def _read_axes(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        try:
            reading = self._get_sensor()._sh3001_getimudata()
        except ImuReadError:
            raise
        except Exception as exc:
            raise ImuReadError("SH3001 read failed") from exc

        if (
            not isinstance(reading, Sequence)
            or isinstance(reading, (str, bytes, bytearray))
            or len(reading) != 2
        ):
            raise ImuReadError("SH3001 reading must contain accelerometer and gyroscope axes")
        return _finite_triple(reading[0], "accelerometer"), _finite_triple(
            reading[1], "gyroscope"
        )

    def read_sample(self, batch_size: int = 1) -> ImuSample:
        """Acquire and average a non-empty batch of fresh direct readings."""
        if isinstance(batch_size, bool) or not isinstance(batch_size, int) or batch_size <= 0:
            raise ImuReadError("batch_size must be a positive integer")

        accelerometer: list[tuple[float, float, float]] = []
        gyroscope: list[tuple[float, float, float]] = []
        for _ in range(batch_size):
            acc_axes, gyro_axes = self._read_axes()
            accelerometer.append(acc_axes)
            gyroscope.append(gyro_axes)

        acc = tuple(
            sum(reading[axis] for reading in accelerometer) / batch_size
            for axis in range(3)
        )
        gyro = tuple(
            sum(reading[axis] for reading in gyroscope) / batch_size
            for axis in range(3)
        )
        ax, ay, az = acc
        pitch_deg = math.degrees(math.atan2(-ay, math.hypot(ax, -az)))
        roll_deg = math.degrees(math.atan2(-az, math.hypot(ax, ay)))

        monotonic_s = _finite_clock_value(self._monotonic(), "monotonic")
        utc_timestamp = self._utc_now()
        if not isinstance(utc_timestamp, str) or not utc_timestamp:
            raise ImuReadError("utc_now must return a non-empty timestamp string")

        return ImuSample(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            accel_xyz=acc,
            gyro_xyz=gyro,
            monotonic_s=monotonic_s,
            utc_timestamp=utc_timestamp,
            valid=True,
        )

    def diagnose_freshness(
        self,
        duration_s: float = 2.0,
        minimum_samples: int = 20,
    ) -> ImuFreshnessReport:
        """Read continuously for a duration and report timestamp freshness."""
        if (
            isinstance(duration_s, bool)
            or not isinstance(duration_s, Real)
            or not math.isfinite(duration_s)
            or duration_s <= 0
        ):
            raise ImuReadError("duration_s must be a positive finite number")
        if (
            isinstance(minimum_samples, bool)
            or not isinstance(minimum_samples, int)
            or minimum_samples <= 0
        ):
            raise ImuReadError("minimum_samples must be a positive integer")

        started = _finite_clock_value(self._monotonic(), "monotonic")
        timestamps: list[float] = []
        while True:
            sample = self.read_sample()
            timestamps.append(sample.monotonic_s)
            ended = _finite_clock_value(self._monotonic(), "monotonic")
            if ended < started:
                raise ImuReadError("monotonic clock moved backwards")
            if ended - started >= float(duration_s):
                break

        duration = ended - started
        first = timestamps[0]
        last = timestamps[-1]
        timestamps_advance = last > first
        estimated_hz = (
            (len(timestamps) - 1) / (last - first)
            if len(timestamps) > 1 and timestamps_advance
            else 0.0
        )
        return ImuFreshnessReport(
            sample_count=len(timestamps),
            duration_s=duration,
            estimated_hz=estimated_hz,
            first_monotonic_s=first,
            last_monotonic_s=last,
            accepted=len(timestamps) >= minimum_samples and timestamps_advance,
        )
