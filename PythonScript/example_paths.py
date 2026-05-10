#!/usr/bin/env python3
"""Shared path helpers for the example scripts."""

from pathlib import Path
import os


SDK_ROOT_ENV_VAR = "ASEE_GLASSES_SDK_ROOT"
DEFAULT_VENDOR_SDK_ROOT = Path(r"E:/7invensun/aSeeGlassesPlusUserSDK")

SCRIPT_DIR = Path(__file__).resolve().parent
SDK_PROJECT_ROOT = SCRIPT_DIR
LOG_DIR = SDK_PROJECT_ROOT / "log"
CALIBRATION_PROFILE_DIR = SDK_PROJECT_ROOT / "calibration_profiles"


def resolve_sdk_root(explicit_root=None):
    if explicit_root:
        return Path(explicit_root).expanduser()

    env_root = os.environ.get(SDK_ROOT_ENV_VAR)
    if env_root:
        return Path(env_root).expanduser()

    return DEFAULT_VENDOR_SDK_ROOT


def sdk_config_dir(explicit_root=None):
    return resolve_sdk_root(explicit_root) / "bin" / "config"


def add_sdk_root_argument(parser):
    parser.add_argument(
        "--sdk-root",
        default=str(resolve_sdk_root()),
        help=(
            "Vendor SDK root directory. Defaults to the value of "
            f"{SDK_ROOT_ENV_VAR} or {DEFAULT_VENDOR_SDK_ROOT}."
        ),
    )
