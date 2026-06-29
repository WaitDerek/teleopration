from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


DEFAULT_CONFIG_FILE = "config/teleop.env"


def load_env_file(path: str | os.PathLike[str] = DEFAULT_CONFIG_FILE) -> dict[str, str]:
    config_path = Path(path)
    values: dict[str, str] = {}
    if not config_path.exists():
        return values
    for raw_line in config_path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def config_value(name: str, *, config: Optional[dict[str, str]] = None, default: Optional[str] = None) -> Optional[str]:
    if name in os.environ:
        return os.environ[name]
    values = load_env_file() if config is None else config
    return values.get(name, default)


def config_int(name: str, *, config: Optional[dict[str, str]] = None, default: int) -> int:
    value = config_value(name, config=config, default=None)
    return default if value is None or value == "" else int(value)


def local_client_url(public_host: Optional[str], port: int) -> Optional[str]:
    if not public_host:
        return None
    return f"https://{public_host}:{port}"
