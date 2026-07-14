#!/usr/bin/env python3
"""
Build the MSAP1_RPU R5 application components with Vitis.

Run with Vitis Python from the MSAP1_RPU directory:

    vitis -s scripts/build_r5_apps.py -- r5c0
    vitis -s scripts/build_r5_apps.py -- r5c1
    vitis -s scripts/build_r5_apps.py -- all
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import vitis


PROJECT_ROOT = Path(__file__).resolve().parents[1]
APP_COMPONENTS = {
    "r5c0": "R5c0",
    "r5c1": "R5c1",
}


def parse_args() -> argparse.Namespace:
    argv = sys.argv[1:]
    if argv and argv[0] == "--":
        argv = argv[1:]

    parser = argparse.ArgumentParser(
        description="Build the MSAP1_RPU R5c0, R5c1, or both app components.",
    )
    parser.add_argument(
        "target",
        metavar="{r5c0,r5c1,all}",
        help="Application component to build. The target is case-insensitive.",
    )
    parser.add_argument(
        "--workspace",
        default=str(PROJECT_ROOT),
        help=f"Vitis workspace/project root. Default: {PROJECT_ROOT}",
    )
    return parser.parse_args(argv)


def require_path(path: Path, description: str) -> Path:
    path = path.expanduser().resolve()
    if not path.exists():
        raise SystemExit(f"Missing {description}: {path}")
    return path


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


def target_components(target: str) -> list[str]:
    target = target.lower()
    if target == "all":
        return [APP_COMPONENTS["r5c0"], APP_COMPONENTS["r5c1"]]
    if target in APP_COMPONENTS:
        return [APP_COMPONENTS[target]]

    choices = ", ".join(("r5c0", "r5c1", "all"))
    raise SystemExit(f"Unknown target '{target}'. Expected one of: {choices}")


def main() -> int:
    args = parse_args()
    workspace = require_path(Path(args.workspace), "workspace")

    client = vitis.create_client()
    try:
        status = set_vitis_workspace(client, workspace)
        print(f"set workspace -> {status}")

        for component_name in target_components(args.target):
            component_dir = workspace / component_name
            require_path(component_dir, f"{component_name} component")

            print(f"Building {component_name}")
            component = client.get_component(name=component_name)
            status = component.build()
            print(f"{component_name}.build() -> {status}")

        return 0
    finally:
        vitis.dispose()


if __name__ == "__main__":
    raise SystemExit(main())
