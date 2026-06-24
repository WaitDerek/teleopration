from __future__ import annotations

from pathlib import Path
from typing import Optional


def show_open_url(url: Optional[str], label: str = "Open URL on Vision Pro") -> None:
    if not url:
        return
    print(f"{label}: {url}")
    latest_path = Path("recordings") / "latest_avp_url.txt"
    try:
        latest_path.parent.mkdir(parents=True, exist_ok=True)
        latest_path.write_text(url + "\n", encoding="utf-8")
        print(f"Saved current URL to: {latest_path}")
    except OSError as exc:
        print(f"Warning: could not write {latest_path}: {exc}")
