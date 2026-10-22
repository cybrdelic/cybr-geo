"""Use official Ubuntu HTTPS archives on disposable CI runners.

GitHub's Azure mirror can keep downloading too slowly to trip APT's inactivity
timeout: https://github.com/actions/runner-images/issues/14823 . Changing only
repository URIs preserves suites, components, Signed-By and APT verification.
"""
from __future__ import annotations

import os
from pathlib import Path
import re
import sys
from urllib.parse import urlsplit


MIRROR_URI = "mirror+file:/etc/apt/apt-mirrors.txt"
OFFICIAL_HOSTS = {"archive.ubuntu.com", "security.ubuntu.com"}
AZURE_HOST = "azure.archive.ubuntu.com"
UBUNTU_URI = re.compile(
    r"https?://(azure\.archive|archive|security)\.ubuntu\.com/ubuntu/?(?=\s|$)"
)


def official_uri(match: re.Match) -> str:
    host = "security" if match[1] == "security" else "archive"
    return f"https://{host}.ubuntu.com/ubuntu/"


def configure(apt_dir: Path) -> list[Path]:
    """Edit recognized Ubuntu sources only; never weaken repository trust."""
    files = [apt_dir / "sources.list"]
    for suffix in ("*.list", "*.sources"):
        files.extend(sorted((apt_dir / "sources.list.d").glob(suffix)))
    original = {path: path.read_text() for path in files if path.is_file()}
    active_sources = ["\n".join(line.split("#", 1)[0] for line in text.splitlines())
                      for text in original.values()]
    if not any(UBUNTU_URI.search(text) or MIRROR_URI in text
               for text in active_sources):
        raise RuntimeError("No supported Ubuntu source URI found; inspect runner configuration")
    replacements = {path: UBUNTU_URI.sub(official_uri, text)
                    for path, text in original.items()}
    if any(MIRROR_URI in text for text in active_sources):
        path = apt_dir / "apt-mirrors.txt"
        original[path] = path.read_text()
        kept = []
        for line in original[path].splitlines():
            if not line.strip() or line.lstrip().startswith("#"):
                kept.append(line)
                continue
            uri = urlsplit(line.split()[0])
            if (uri.scheme not in {"http", "https"}
                    or uri.netloc not in OFFICIAL_HOSTS | {AZURE_HOST}
                    or uri.path.rstrip("/") != "/ubuntu"
                    or uri.query or uri.fragment):
                raise RuntimeError(f"Unexpected mirror in {path}; inspect runner configuration")
            if uri.netloc != AZURE_HOST:
                kept.append(UBUNTU_URI.sub(official_uri, line))
        if not any(UBUNTU_URI.search(line.split("#", 1)[0]) for line in kept):
            kept.extend(("https://archive.ubuntu.com/ubuntu/\tpriority:1",
                         "https://security.ubuntu.com/ubuntu/\tpriority:2"))
        replacements[path] = "\n".join(kept) + "\n"
    changed = []
    # Validate the complete input before writing any source file.
    for path, text in replacements.items():
        if text != original[path]:
            path.write_text(text)
            changed.append(path)
    return changed


if __name__ == "__main__":
    if sys.platform != "linux" or os.geteuid() != 0:
        raise SystemExit("Run with sudo on an Ubuntu CI runner")
    if 'ID=ubuntu' not in Path("/etc/os-release").read_text().splitlines():
        raise SystemExit("Only Ubuntu CI runners are supported")
    changed = configure(Path("/etc/apt"))
    print("Official Ubuntu HTTPS sources configured: "
          + (", ".join(map(str, changed)) if changed else "already configured"))
