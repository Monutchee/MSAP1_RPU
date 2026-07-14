#!/usr/bin/env python3
"""
Create the MSAP1_RPU Vitis platform from an XSA.

Run with Vitis Python from the MSAP1_RPU directory:

    mkdir -p /tmp/xilinx-vitis-data
    export XILINX_VITIS_DATA_DIR=/tmp/xilinx-vitis-data
    vitis -s scripts/create_platform_from_xsa.py -- --force

By default the script expects the XSA at:

    ../runtime-generated/bin_file/MSAP1_PL.xsa
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
from pathlib import Path

import vitis


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_XSA = PROJECT_ROOT.parent / "runtime-generated" / "bin_file" / "MSAP1_PL.xsa"

PLATFORM_NAME = "platform"

OS_CONFIG_PARAMS = (
    "freertos_timer_select",
    "freertos_timer_select_counter",
)

LIB_CONFIG_PREFIXES = {
    "XILTIMER_": "xiltimer",
}

DOMAINS = {
    "freertos_psu_cortexr5_0": {
        "cpu": "psu_cortexr5_0",
        "XILTIMER_sleep_timer": "Default",
        "XILTIMER_tick_timer": "psu_ttc_6",
        "tick_base": "0xff130000",
        "freertos_timer_select": "psu_ttc_6",
        "freertos_timer_select_counter": "0x0",
        "freertos_base": "0xff130000",
    },
    "freertos_psu_cortexr5_1": {
        "cpu": "psu_cortexr5_1",
        "XILTIMER_sleep_timer": "Default",
        "XILTIMER_tick_timer": "psu_ttc_3",
        "tick_base": "0xff120000",
        "freertos_timer_select": "psu_ttc_3",
        "freertos_timer_select_counter": "0x0",
        "freertos_base": "0xff120000",
    },
}


def parse_args() -> argparse.Namespace:
    argv = sys.argv[1:]
    if argv and argv[0] == "--":
        argv = argv[1:]

    parser = argparse.ArgumentParser(
        description="Create and build the MSAP1_RPU Vitis platform from an XSA.",
    )
    parser.add_argument(
        "--xsa",
        default=str(DEFAULT_XSA),
        help=f"Path to the hardware XSA. Default: {DEFAULT_XSA}",
    )
    parser.add_argument(
        "--workspace",
        default=str(PROJECT_ROOT),
        help=f"Vitis workspace/project root. Default: {PROJECT_ROOT}",
    )
    parser.add_argument(
        "--vitis-install",
        default=os.environ.get("XILINX_VITIS", "/opt/Xilinx/2025.2/Vitis"),
        help="Vitis install directory. Default: $XILINX_VITIS or /opt/Xilinx/2025.2/Vitis",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Remove an existing platform component before recreating it.",
    )
    parser.add_argument(
        "--skip-app-build",
        action="store_true",
        help="Only build the platform, not the R5c0/R5c1 app components.",
    )
    return parser.parse_args(argv)


def require_path(path: Path, description: str) -> Path:
    path = path.expanduser().resolve()
    if not path.exists():
        raise SystemExit(f"Missing {description}: {path}")
    return path


def embeddedsw_root(vitis_install: Path) -> Path:
    candidates = [
        vitis_install / "data" / "embeddedsw",
        vitis_install.parent / "data" / "embeddedsw",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise SystemExit(
        "Could not find embeddedsw. Checked:\n"
        + "\n".join(f"  {candidate}" for candidate in candidates)
    )


def set_vitis_workspace(client, workspace: Path):
    try:
        return client.set_workspace(path=str(workspace))
    except Exception as exc:
        message = str(exc)
        needs_update = (
            "workspace version" in message
            or "Click 'Update'" in message
            or "initialize this folder as a Vitis IDE workspace" in message
        )
        if not needs_update:
            raise

        print(f"Initializing/updating Vitis workspace metadata: {workspace}")
        return client.update_workspace(path=str(workspace))


def iter_config_keys(config: dict[str, str], keys: tuple[str, ...]) -> list[str]:
    return [key for key in keys if key in config]


def iter_config_keys_with_prefix(config: dict[str, str], prefix: str) -> list[str]:
    return [key for key in config if key.startswith(prefix)]


def set_os_configs(domain_name: str, domain, config: dict[str, str]) -> None:
    for param in iter_config_keys(config, OS_CONFIG_PARAMS):
        value = config[param]
        print(f"Configuring {domain_name}: {param} -> {value}")
        status = domain.set_config(option="os", param=param, value=value)
        print(f"  set_config({param}) -> {status}")


def set_lib_configs(domain_name: str, domain, config: dict[str, str]) -> None:
    for prefix, lib_name in LIB_CONFIG_PREFIXES.items():
        for param in iter_config_keys_with_prefix(config, prefix):
            value = config[param]
            print(f"Configuring {domain_name}: {param} -> {value}")
            status = domain.set_config(
                option="lib",
                lib_name=lib_name,
                param=param,
                value=value,
            )
            print(f"  set_config({param}) -> {status}")


def configure_domain(domain_name: str, domain, libs: dict[str, Path], config: dict[str, str]) -> None:
    for lib_name in ("libmetal", "openamp", "xilmailbox"):
        lib_path = libs[lib_name]
        print(f"Configuring {domain_name}: {lib_name} -> {lib_path}")
        status = domain.set_lib(lib_name=lib_name, path=str(lib_path))
        print(f"  set_lib({lib_name}) -> {status}")

    set_os_configs(domain_name, domain, config)
    set_lib_configs(domain_name, domain, config)


def require_define(text: str, header: Path, symbol: str, expected_value: str) -> None:
    pattern = rf"^\s*#define\s+{re.escape(symbol)}\s+{re.escape(expected_value)}\b"
    if not re.search(pattern, text, flags=re.MULTILINE | re.IGNORECASE):
        raise ValueError(f"expected #define {symbol} {expected_value} in {header}")


def verify_timer_headers(workspace: Path) -> None:
    sw_dir = workspace / PLATFORM_NAME / "export" / PLATFORM_NAME / "sw"
    failures: list[str] = []

    for domain_name, config in DOMAINS.items():
        include_dir = sw_dir / domain_name / "include"

        xiltimer_header = include_dir / "xtimer_config.h"
        freertos_header = include_dir / "FreeRTOSConfig.h"
        missing_header = False
        for header in (xiltimer_header, freertos_header):
            if not header.exists():
                failures.append(f"{domain_name}: missing {header}")
                missing_header = True
        if missing_header:
            continue

        try:
            text = xiltimer_header.read_text(encoding="utf-8", errors="replace")
            require_define(text, xiltimer_header, "XTIMER_IS_DEFAULT_TIMER", "1")
            require_define(text, xiltimer_header, "XTICKTIMER_BASEADDRESS", config["tick_base"])
            print(
                f"Verified {domain_name}: XTIMER_IS_DEFAULT_TIMER 1, "
                f"XTICKTIMER_BASEADDRESS {config['tick_base']}"
            )
        except ValueError as exc:
            failures.append(f"{domain_name}: {exc}")

        try:
            text = freertos_header.read_text(encoding="utf-8", errors="replace")
            require_define(text, freertos_header, "configTIMER_BASEADDR", config["freertos_base"])
            require_define(
                text,
                freertos_header,
                "configTIMER_SELECT_CNTR",
                config["freertos_timer_select_counter"],
            )
            print(
                f"Verified {domain_name}: configTIMER_BASEADDR {config['freertos_base']}, "
                f"configTIMER_SELECT_CNTR {config['freertos_timer_select_counter']}"
            )
        except ValueError as exc:
            failures.append(f"{domain_name}: {exc}")

    if failures:
        raise SystemExit("Timer verification failed:\n" + "\n".join(f"  {item}" for item in failures))


def main() -> int:
    args = parse_args()

    workspace = require_path(Path(args.workspace), "workspace")
    xsa = require_path(Path(args.xsa), "XSA")
    vitis_install = require_path(Path(args.vitis_install), "Vitis install")
    embeddedsw = embeddedsw_root(vitis_install)

    libs = {
        "libmetal": embeddedsw / "ThirdParty" / "sw_services" / "libmetal_v2_9",
        "openamp": embeddedsw / "ThirdParty" / "sw_services" / "openamp_v1_12",
        "xilmailbox": embeddedsw / "lib" / "sw_services" / "xilmailbox_v1_12",
    }
    for lib_name, lib_path in libs.items():
        require_path(lib_path, f"{lib_name} library")

    platform_dir = workspace / PLATFORM_NAME
    if platform_dir.exists():
        if not args.force:
            raise SystemExit(
                f"{platform_dir} already exists. Re-run with --force to recreate it."
            )
        print(f"Removing existing platform component: {platform_dir}")
        shutil.rmtree(platform_dir)

        wsdata = workspace / "_ide" / ".wsdata"
        if wsdata.exists():
            print(f"Removing stale Vitis workspace metadata: {wsdata}")
            shutil.rmtree(wsdata)

    client = vitis.create_client()
    try:
        status = set_vitis_workspace(client, workspace)
        print(f"set workspace -> {status}")

        print(f"Creating platform '{PLATFORM_NAME}' from XSA: {xsa}")
        platform = client.create_platform_component(
            name=PLATFORM_NAME,
            hw_design=str(xsa),
            os="freertos",
            cpu=DOMAINS["freertos_psu_cortexr5_0"]["cpu"],
            domain_name="freertos_psu_cortexr5_0",
            no_boot_bsp=True,
        )

        print("Adding freertos_psu_cortexr5_1 domain")
        platform.add_domain(
            cpu=DOMAINS["freertos_psu_cortexr5_1"]["cpu"],
            os="freertos",
            name="freertos_psu_cortexr5_1",
            display_name="freertos_psu_cortexr5_1",
            generate_dtb=False,
            hw_boot_bin="",
        )

        platform = client.get_component(name=PLATFORM_NAME)
        for domain_name, config in DOMAINS.items():
            domain = platform.get_domain(name=domain_name)
            configure_domain(domain_name, domain, libs, config)
            print(f"Regenerating {domain_name} BSP sources")
            status = domain.regenerate()
            print(f"  regenerate({domain_name}) -> {status}")

        print("Building platform")
        status = platform.build()
        print(f"platform.build() -> {status}")

        verify_timer_headers(workspace)

        if not args.skip_app_build:
            for component_name in ("R5c0", "R5c1"):
                print(f"Building {component_name}")
                component = client.get_component(name=component_name)
                status = component.build()
                print(f"{component_name}.build() -> {status}")

        return 0
    finally:
        vitis.dispose()


if __name__ == "__main__":
    raise SystemExit(main())
