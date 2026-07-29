#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS_DIR))

from vitis_status import require_vitis_success  # noqa: E402


class VitisStatusTests(unittest.TestCase):
    def test_accepts_documented_success_forms(self) -> None:
        for status in (None, 0, True, "0", "ok", "success", "succeeded"):
            with self.subTest(status=status):
                require_vitis_success("operation", status)

    def test_rejects_nonzero_build_status(self) -> None:
        with self.assertRaisesRegex(
            RuntimeError,
            r"R5c1\.build\(\) failed \(status 1\)",
        ):
            require_vitis_success("R5c1.build()", 1)

    def test_rejects_unknown_or_false_status(self) -> None:
        for status in (False, "failed", "error", object()):
            with self.subTest(status=status):
                with self.assertRaises(RuntimeError):
                    require_vitis_success("operation", status)


if __name__ == "__main__":
    unittest.main()
