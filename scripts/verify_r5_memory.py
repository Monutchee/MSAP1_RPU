#!/usr/bin/env python3
"""Post-link memory and stack gate for the MSAP1 R5 firmware images."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path


MINIMUM_UNUSED_BYTES = 1 * 1024 * 1024
MAXIMUM_M17_STACK_FRAME = 1024
REQUIRED_DDR_SECTIONS = (".text", ".rodata", ".data", ".bss", ".heap", ".stack")
M17_STACK_SOURCES = ("energy_demand_engine.cpp", "r5_session_id.cpp")


class GateError(RuntimeError):
    """A firmware image violated a post-link safety invariant."""


@dataclass(frozen=True)
class Section:
    name: str
    address: int
    size: int
    flags: str

    @property
    def end(self) -> int:
        return self.address + self.size


@dataclass(frozen=True)
class Symbol:
    address: int
    size: int
    kind: str
    name: str


SECTION_PATTERN = re.compile(
    r"^\s*\[\s*\d+\]\s+(\S+)\s+\S+\s+([0-9a-fA-F]+)\s+"
    r"[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+\S+\s+([A-Z]*)\b"
)
SYMBOL_PATTERN = re.compile(
    r"^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([A-Za-z])\s+(.*)$"
)


def parse_sections(output: str) -> dict[str, Section]:
    result: dict[str, Section] = {}
    for line in output.splitlines():
        match = SECTION_PATTERN.match(line)
        if not match:
            continue
        name, address, size, flags = match.groups()
        result[name] = Section(name, int(address, 16), int(size, 16), flags)
    return result


def parse_symbols(output: str) -> list[Symbol]:
    result: list[Symbol] = []
    for line in output.splitlines():
        match = SYMBOL_PATTERN.match(line)
        if match:
            address, size, kind, name = match.groups()
            result.append(Symbol(int(address, 16), int(size, 16), kind, name))
    return result


def parse_stack_usage(paths: list[Path]) -> tuple[int, int, str]:
    count = 0
    maximum = 0
    maximum_name = ""
    for path in paths:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if not any(source in line for source in M17_STACK_SOURCES):
                continue
            fields = line.rsplit("\t", 2)
            if len(fields) < 2:
                continue
            try:
                frame = int(fields[-2] if len(fields) == 3 else fields[-1])
            except ValueError:
                continue
            count += 1
            if frame >= maximum:
                maximum = frame
                maximum_name = fields[0]
    return count, maximum, maximum_name


def command_output(command: list[str]) -> str:
    try:
        return subprocess.run(
            command, check=True, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        ).stdout
    except (OSError, subprocess.CalledProcessError) as exc:
        raise GateError(f"cannot run {' '.join(command)}: {exc}") from exc


def firmware_region(contract_path: Path, core_id: str) -> tuple[int, int]:
    document = json.loads(contract_path.read_text(encoding="utf-8"))
    for core in document.get("cores", []):
        if core.get("id") == core_id:
            firmware = core["firmware"]
            return int(firmware["start"], 0), int(firmware["size"], 0)
    raise GateError(f"OpenAMP contract has no core {core_id}")


def verify_sections(sections: dict[str, Section], start: int, size: int) -> tuple[int, int]:
    end = start + size
    missing = [name for name in REQUIRED_DDR_SECTIONS if name not in sections]
    if missing:
        raise GateError("ELF is missing required sections: " + ", ".join(missing))
    for name in REQUIRED_DDR_SECTIONS:
        section = sections[name]
        if section.address < start or section.end > end:
            raise GateError(
                f"{name} 0x{section.address:x}..0x{section.end:x} is outside "
                f"firmware 0x{start:x}..0x{end:x}"
            )
    # Include every allocated section placed in the firmware window, not only
    # the six large named sections. This catches resource tables, exception
    # tables, constructor arrays, and future linker additions that happen to
    # extend beyond .stack. Allocated TCM/OCM sections remain outside this DDR
    # calculation by design.
    firmware_allocated: list[Section] = []
    for section in sections.values():
        if "A" not in section.flags or section.size == 0:
            continue
        overlaps = section.address < end and section.end > start
        if not overlaps:
            continue
        if section.address < start or section.end > end:
            raise GateError(
                f"allocated {section.name} 0x{section.address:x}.."
                f"0x{section.end:x} crosses firmware boundary "
                f"0x{start:x}..0x{end:x}"
            )
        firmware_allocated.append(section)
    if not firmware_allocated:
        raise GateError("ELF has no allocated section in the firmware region")
    post_link_end = max(section.end for section in firmware_allocated)
    unused = end - post_link_end
    if unused < MINIMUM_UNUSED_BYTES:
        raise GateError(
            f"only {unused} bytes remain after code/static/heap/stacks; "
            f"at least {MINIMUM_UNUSED_BYTES} are required"
        )
    return post_link_end, unused


def verify_static_symbol(
    symbols: list[Symbol], sections: dict[str, Section], requested: str,
) -> Symbol:
    matches = [symbol for symbol in symbols if symbol.name == requested]
    if len(matches) != 1:
        raise GateError(
            f"expected one linker symbol named {requested!r}, found {len(matches)}"
        )
    symbol = matches[0]
    if symbol.kind not in "bBdD":
        raise GateError(f"{requested} is not a .bss/.data symbol (type {symbol.kind})")
    allowed = [sections[name] for name in (".bss", ".data") if name in sections]
    if not any(section.address <= symbol.address and symbol.address + symbol.size <= section.end
               for section in allowed):
        raise GateError(f"{requested} is not wholly located in .bss or .data")
    return symbol


def resolve_tool(value: str) -> str:
    resolved = shutil.which(value)
    if not resolved:
        raise GateError(f"required ELF tool is unavailable: {value}")
    return resolved


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--core", choices=("r5c0", "r5c1"), required=True)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--readelf", default="readelf")
    parser.add_argument("--nm", default="nm")
    parser.add_argument("--stack-usage-dir", type=Path)
    parser.add_argument("--require-static-symbol", action="append", default=[])
    args = parser.parse_args()

    try:
        for path, label in ((args.contract, "contract"), (args.elf, "ELF")):
            if not path.is_file():
                raise GateError(f"missing {label}: {path}")
        start, size = firmware_region(args.contract, args.core)
        sections = parse_sections(command_output([
            resolve_tool(args.readelf), "-S", "-W", str(args.elf),
        ]))
        post_link_end, unused = verify_sections(sections, start, size)
        symbols = parse_symbols(command_output([
            resolve_tool(args.nm), "-S", "--demangle", str(args.elf),
        ]))
        for requested in args.require_static_symbol:
            symbol = verify_static_symbol(symbols, sections, requested)
            print(
                f"{args.core}: {requested} is static at 0x{symbol.address:x} "
                f"({symbol.size} bytes, type {symbol.kind})"
            )

        if args.core == "r5c1":
            if not args.stack_usage_dir:
                raise GateError("r5c1 requires --stack-usage-dir")
            files = sorted(args.stack_usage_dir.rglob("*.su"))
            count, maximum, function = parse_stack_usage(files)
            if count == 0:
                raise GateError(
                    "no M17 .su entries found; compile R5C1 with -fstack-usage"
                )
            if maximum >= MAXIMUM_M17_STACK_FRAME:
                raise GateError(
                    f"M17 stack frame is {maximum} bytes in {function}; "
                    f"must be below {MAXIMUM_M17_STACK_FRAME}"
                )
            print(
                f"{args.core}: {count} M17 stack frames checked; maximum "
                f"{maximum} bytes ({function})"
            )

        print(
            f"{args.core}: post-link end 0x{post_link_end:x}; "
            f"unused {unused} bytes of {size}"
        )
        return 0
    except (GateError, KeyError, ValueError, json.JSONDecodeError) as exc:
        print(f"r5-memory-gate: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
