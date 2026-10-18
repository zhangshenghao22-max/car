from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import time
import unittest
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "board_linux_uploader.py"
SPEC = importlib.util.spec_from_file_location("board_linux_uploader_async_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class AsyncInspectionUploadTests(unittest.TestCase):
    @staticmethod
    def manager(status_path: Path):
        return MODULE.CabinetNavTakeoverManager(
            SimpleNamespace(
                enable_cabinet_nav_takeover=False,
                cabinet_takeover_status_file=str(status_path),
                server_url="http://127.0.0.1:9",
                upload_token="test-token",
                board_id="test-board",
                board_label="Test Board",
            )
        )

    def test_outbox_upload_runs_without_blocking_tick(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            outbox = root / "outbox"
            outbox.mkdir()
            queued_path = outbox / "inspection-1_inspection.json"
            queued_path.write_text(
                json.dumps(
                    {
                        "kind": "inspection",
                        "inspection_id": "inspection-1",
                        "payload": {"inspection_id": "inspection-1", "status": "completed"},
                    }
                ),
                encoding="utf-8",
            )
            manager = self.manager(root / "status.json")

            def delayed_upload(path, record):
                time.sleep(0.25)
                return path, str(record["kind"])

            try:
                with mock.patch.object(MODULE, "CABINET_TAKEOVER_OUTBOX_DIR", outbox):
                    with mock.patch.object(manager, "_upload_outbox_record", side_effect=delayed_upload):
                        started = time.monotonic()
                        manager._retry_outbox()
                        elapsed = time.monotonic() - started
                        self.assertLess(elapsed, 0.1)
                        self.assertTrue(queued_path.exists())
                        self.assertEqual(len(manager._outbox_futures), 1)

                        time.sleep(0.3)
                        manager._retry_outbox()
                        self.assertFalse(queued_path.exists())
                        self.assertEqual(manager.outbox_uploaded, 1)
                        self.assertEqual(len(manager._outbox_futures), 0)
                        self.assertTrue(manager.outbox_last_uploaded_at)
            finally:
                manager.close()


if __name__ == "__main__":
    unittest.main()
