import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "verify_r5_memory.py"
SPEC = importlib.util.spec_from_file_location("verify_r5_memory", MODULE_PATH)
gate = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = gate
SPEC.loader.exec_module(gate)


class R5MemoryGateTests(unittest.TestCase):
    def test_section_headroom_and_static_symbol(self):
        sections = {
            name: gate.Section(name, 0x40000000 + index * 0x1000, 0x800, "WA")
            for index, name in enumerate(gate.REQUIRED_DDR_SECTIONS)
        }
        sections[".resource_table"] = gate.Section(
            ".resource_table", 0x40008000, 0x100, "A"
        )
        end, unused = gate.verify_sections(sections, 0x40000000, 0x800000)
        self.assertEqual(end, sections[".resource_table"].end)
        self.assertGreater(unused, gate.MINIMUM_UNUSED_BYTES)
        symbol = gate.verify_static_symbol(
            [gate.Symbol(sections[".bss"].address, 0x100, "b", "aggregation_engine")],
            sections,
            "aggregation_engine",
        )
        self.assertEqual(symbol.kind, "b")

    def test_rejects_insufficient_headroom(self):
        sections = {
            name: gate.Section(name, 0x40000000, 0x100, "WA")
            for name in gate.REQUIRED_DDR_SECTIONS
        }
        sections[".stack"] = gate.Section(".stack", 0x40780000, 0x10000, "WA")
        with self.assertRaises(gate.GateError):
            gate.verify_sections(sections, 0x40000000, 0x800000)

    def test_parses_only_guarded_aggregation_stack_usage(self):
        with tempfile.TemporaryDirectory() as directory:
            usage = Path(directory) / "aggregation.su"
            usage.write_text(
                "x/energy_demand_engine.cpp:10:1:observe\t160\tstatic\n"
                "x/other.cpp:10:1:large\t4096\tstatic\n"
                "x/r5_session_id.cpp:4:1:nonce\t32\tstatic\n"
                "x/flicker_engine.cpp:8:1:process\t312\tstatic\n"
                "x/mains_signal_engine.cpp:9:1:process\t304\tstatic\n"
                "x/voltage_sample_frame_decoder.cpp:10:1:decode\t56\tstatic\n"
                "x/aggregation_shadow_service.cpp:11:1:validate\t336\tstatic\n",
                encoding="utf-8",
            )
            count, maximum, function = gate.parse_stack_usage([usage])
            self.assertEqual((count, maximum), (6, 336))
            self.assertIn("validate", function)


if __name__ == "__main__":
    unittest.main()
