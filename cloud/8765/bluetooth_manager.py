from __future__ import annotations

import asyncio
import sys
import threading
from dataclasses import dataclass
from pathlib import Path

VENDOR_DIR = Path(__file__).resolve().parent / "_vendor"
if VENDOR_DIR.exists():
    vendor_str = str(VENDOR_DIR)
    if vendor_str not in sys.path:
        sys.path.insert(0, vendor_str)

try:
    from bleak import BleakClient, BleakScanner

    BLEAK_IMPORT_ERROR = None
except Exception as exc:
    BleakClient = None
    BleakScanner = None
    BLEAK_IMPORT_ERROR = exc


DEFAULT_BLE_SCAN_TIMEOUT = 4.0
DEFAULT_BLE_WRITE_UUID = "0000FFE2-0000-1000-8000-00805F9B34FB"
DEFAULT_BLE_NOTIFY_UUID = "0000FFE1-0000-1000-8000-00805F9B34FB"


@dataclass
class BleDeviceInfo:
    name: str
    address: str
    rssi: int | None = None

    @property
    def label(self) -> str:
        parts = [self.name or "Unknown", self.address]
        if self.rssi is not None:
            parts.append(f"RSSI {self.rssi}")
        return " | ".join(parts)


class BleUartManager:
    def __init__(self, *, log_callback=None, rx_callback=None):
        self.log_callback = log_callback
        self.rx_callback = rx_callback
        self._loop_ready = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None
        self._client = None
        self._notify_uuid: str | None = None
        self._device_name = ""
        self._device_address = ""
        self._lock = threading.Lock()

        self.available = BleakClient is not None and BleakScanner is not None
        self.import_error = BLEAK_IMPORT_ERROR

        if self.available:
            self._ensure_loop()

    def _log(self, message: str):
        if self.log_callback is not None:
            self.log_callback(message)

    def _ensure_loop(self):
        if self._loop_thread is not None and self._loop_thread.is_alive():
            return
        self._loop_thread = threading.Thread(target=self._loop_worker, daemon=True)
        self._loop_thread.start()
        self._loop_ready.wait(timeout=3.0)

    def _loop_worker(self):
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        self._loop_ready.set()
        loop.run_forever()

        pending = asyncio.all_tasks(loop)
        for task in pending:
            task.cancel()
        if pending:
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
        loop.close()

    def _run_coro(self, coro):
        if not self.available:
            raise RuntimeError(f"Bleak unavailable: {self.import_error}")
        self._ensure_loop()
        if self._loop is None:
            raise RuntimeError("Bluetooth event loop not ready")
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result()

    async def _scan_async(self, timeout: float) -> list[BleDeviceInfo]:
        devices = await BleakScanner.discover(timeout=timeout)
        result: list[BleDeviceInfo] = []
        for device in devices:
            result.append(
                BleDeviceInfo(
                    name=device.name or "",
                    address=device.address,
                    rssi=getattr(device, "rssi", None),
                )
            )
        result.sort(key=lambda item: ((item.name or "").lower(), item.address))
        return result

    def scan(self, timeout: float = DEFAULT_BLE_SCAN_TIMEOUT) -> list[BleDeviceInfo]:
        return self._run_coro(self._scan_async(timeout))

    def _notification_handler(self, _sender, data: bytearray):
        if self.rx_callback is None:
            return
        try:
            text = bytes(data).decode("utf-8", errors="ignore").strip()
        except Exception:
            text = ""
        if text:
            self.rx_callback(text)

    async def _connect_async(
        self,
        *,
        address: str,
        write_uuid: str,
        notify_uuid: str | None = None,
    ) -> tuple[bool, str]:
        await self._disconnect_async()

        client = BleakClient(address)
        await client.connect()
        if not client.is_connected:
            return False, f"Bluetooth connect failed: {address}"

        if notify_uuid:
            await client.start_notify(notify_uuid, self._notification_handler)

        with self._lock:
            self._client = client
            self._notify_uuid = notify_uuid or None
            self._device_address = address
            self._device_name = address
            self._write_uuid = write_uuid

        return True, address

    def connect(
        self,
        *,
        address: str,
        write_uuid: str,
        notify_uuid: str | None = None,
    ) -> tuple[bool, str]:
        return self._run_coro(
            self._connect_async(
                address=address,
                write_uuid=write_uuid,
                notify_uuid=notify_uuid,
            )
        )

    async def _disconnect_async(self):
        with self._lock:
            client = self._client
            notify_uuid = self._notify_uuid
            self._client = None
            self._notify_uuid = None
            self._device_name = ""
            self._device_address = ""

        if client is None:
            return

        try:
            if notify_uuid:
                await client.stop_notify(notify_uuid)
        except Exception:
            pass

        try:
            await client.disconnect()
        except Exception:
            pass

    def disconnect(self):
        if (
            not self.available
            or self._loop is None
            or self._loop_thread is None
            or not self._loop_thread.is_alive()
            or self._loop.is_closed()
        ):
            return
        self._run_coro(self._disconnect_async())

    async def _send_async(self, payload: bytes, *, response: bool) -> tuple[bool, str]:
        with self._lock:
            client = self._client
            write_uuid = getattr(self, "_write_uuid", "")

        if client is None or not client.is_connected:
            return False, "Bluetooth not connected"
        if not write_uuid:
            return False, "Bluetooth write UUID is empty"

        await client.write_gatt_char(write_uuid, payload, response=response)
        return True, "ok"

    def send_text(self, text: str, *, response: bool = False) -> tuple[bool, str]:
        payload = text.encode("ascii", errors="ignore")
        return self._run_coro(self._send_async(payload, response=response))

    def is_connected(self) -> bool:
        with self._lock:
            client = self._client
            return bool(client is not None and client.is_connected)

    def current_target(self) -> str:
        with self._lock:
            return self._device_name or self._device_address or ""

    def close(self):
        if self.available and self._loop is not None:
            try:
                self.disconnect()
            except Exception:
                pass
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._loop_thread is not None and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=2.0)
