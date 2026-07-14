"""I2C speech synthesis client adapted from text_speeker_ws/Speech.py."""

import time
from typing import Iterable, Optional


class SpeechClientError(RuntimeError):
    """Raised when the speech module cannot be accessed."""


class SpeechClient:
    """Small blocking driver for the I2C speech synthesis module."""

    DATA_HEAD = 0xFD

    ENCODING_FORMAT = {
        'GB2312': 0x00,
        'GBK': 0x01,
        'BIG5': 0x02,
        'UNICODE': 0x03,
    }

    CHIP_STATUS = {
        'InitSuccessful': 0x4A,
        'CorrectCommand': 0x41,
        'ErrorCommand': 0x45,
        'Busy': 0x4E,
        'Idle': 0x4F,
    }

    READER = {
        'XiaoYan': 3,
        'XuJiu': 51,
        'XuDuo': 52,
        'XiaoPing': 53,
        'DonaldDuck': 54,
        'XuXiaoBao': 55,
    }

    STYLE = {
        'Single': 0,
        'Continue': 1,
    }

    LANGUAGE = {
        'Auto': 0,
        'Chinese': 1,
        'English': 2,
    }

    def __init__(
        self,
        bus_id: int = 5,
        i2c_addr: int = 0x30,
        byte_interval_sec: float = 0.01,
        status_query_delay_sec: float = 0.05,
        wait_after_speak: bool = False,
        logger=None,
    ):
        self._i2c_addr = i2c_addr
        self._byte_interval_sec = max(0.0, byte_interval_sec)
        self._status_query_delay_sec = max(0.0, status_query_delay_sec)
        self._wait_after_speak = wait_after_speak
        self._logger = logger

        try:
            import smbus  # Imported lazily so x86 syntax checks still work.
        except ImportError as err:
            raise SpeechClientError(
                'python3-smbus is not installed or unavailable') from err

        try:
            self._bus = smbus.SMBus(bus_id)
        except Exception as err:
            raise SpeechClientError(
                f'failed to open I2C bus {bus_id}: {err}') from err

    def configure(
        self,
        reader: str = 'XiaoPing',
        volume: int = 10,
        speed: int = 5,
        intonation: int = 5,
        style: str = 'Continue',
        language: str = 'Chinese',
    ) -> None:
        """Configure stable synthesis parameters once at startup."""
        self.set_reader(self.READER.get(reader, self.READER['XiaoPing']))
        self.set_volume(volume)
        self.set_speed(speed)
        self.set_intonation(intonation)
        self.set_style(self.STYLE.get(style, self.STYLE['Continue']))
        self.set_language(self.LANGUAGE.get(language, self.LANGUAGE['Chinese']))

    def speak_and_wait(
        self,
        text: str,
        encoding: str = 'GB2312',
        timeout_sec: float = 20.0,
    ) -> bool:
        """Send text to the speech chip and block until playback is idle."""
        payload = text.encode('gb2312', errors='replace')
        encoding_format = self.ENCODING_FORMAT.get(encoding, 0x00)
        self._send_text_frame(payload, encoding_format)
        if not self._wait_after_speak:
            return True
        return self.wait_idle(timeout_sec)

    def wait_idle(self, timeout_sec: float = 20.0) -> bool:
        """Wait until the chip reports idle."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            status = self.get_status()
            if status == self.CHIP_STATUS['Idle']:
                return True
            time.sleep(0.05)
        self._warn('speech chip wait idle timed out')
        return False

    def get_status(self) -> Optional[int]:
        """Return chip status byte, or None on read failure."""
        try:
            self._write_bytes([self.DATA_HEAD, 0x00, 0x01, 0x21])
            time.sleep(self._status_query_delay_sec)
            return self._bus.read_byte(self._i2c_addr)
        except Exception as err:
            self._warn(f'I2C status read failed: {err}')
            return None

    def set_reader(self, reader: int) -> None:
        self._send_control('m', self._clamp(reader, 0, 255))
        self.wait_idle(5.0)

    def set_volume(self, volume: int) -> None:
        self._send_control('v', self._clamp(volume, 0, 10))
        self.wait_idle(5.0)

    def set_speed(self, speed: int) -> None:
        self._send_control('s', self._clamp(speed, 0, 10))
        self.wait_idle(5.0)

    def set_intonation(self, intonation: int) -> None:
        self._send_control('t', self._clamp(intonation, 0, 10))
        self.wait_idle(5.0)

    def set_style(self, style: int) -> None:
        self._send_control('f', self._clamp(style, 0, 1))
        self.wait_idle(5.0)

    def set_language(self, language: int) -> None:
        self._send_control('g', self._clamp(language, 0, 2))
        self.wait_idle(5.0)

    def _send_text_frame(self, text: bytes, encoding_format: int) -> None:
        self._send_frame(0x01, bytes([encoding_format]) + text)

    def _send_control(self, key: str, value: int = -1) -> None:
        if value >= 0:
            text = f'[{key}{value}]'
        else:
            text = f'[{key}]'
        self._send_text_frame(text.encode('gb2312'), 0x00)

    def _send_frame(self, command: int, payload: bytes) -> None:
        size = len(payload) + 1
        frame = [
            self.DATA_HEAD,
            (size >> 8) & 0xFF,
            size & 0xFF,
            command,
        ]
        frame.extend(payload)
        self._write_frame(frame)

    def _write_frame(self, data: Iterable[int]) -> None:
        frame = [int(value) & 0xFF for value in data]
        if not frame:
            return
        if len(frame) <= 32 and hasattr(self._bus, 'write_i2c_block_data'):
            try:
                self._bus.write_i2c_block_data(
                    self._i2c_addr, frame[0], frame[1:])
                time.sleep(self._byte_interval_sec)
                return
            except Exception as err:
                self._warn(
                    f'I2C block write failed, fallback to byte writes: {err}')
        self._write_bytes(frame)

    def _write_bytes(self, data: Iterable[int]) -> None:
        for value in data:
            try:
                self._bus.write_byte(self._i2c_addr, int(value) & 0xFF)
            except Exception as err:
                raise SpeechClientError(f'I2C write failed: {err}') from err
            time.sleep(self._byte_interval_sec)

    def _warn(self, text: str) -> None:
        if self._logger is not None:
            self._logger.warn(text)

    @staticmethod
    def _clamp(value: int, min_value: int, max_value: int) -> int:
        return min(max(int(value), min_value), max_value)
