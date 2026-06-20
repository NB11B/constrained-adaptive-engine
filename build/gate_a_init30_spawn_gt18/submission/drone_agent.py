"""Swarm submission loader for the packaged CAE runtime.

Swarm's packager includes drone_agent.py, requirements.txt, and model-like
artifacts such as .zip files. The real CAE implementation and native bridge are
packaged in cae_runtime.zip. This loader extracts that runtime safely and
delegates to cae_agent_impl.DroneFlightController.
"""

from __future__ import annotations

import importlib
import sys
import tempfile
import zipfile
from pathlib import Path


_RUNTIME_MODULE = None


def _safe_extract(zip_path: Path, dest: Path) -> None:
    root = dest.resolve()
    with zipfile.ZipFile(zip_path, "r") as zf:
        for member in zf.infolist():
            target = (root / member.filename).resolve()
            if not str(target).startswith(str(root)):
                raise RuntimeError(f"Unsafe path in CAE runtime zip: {member.filename}")
        zf.extractall(root)


def _load_runtime_module():
    global _RUNTIME_MODULE
    if _RUNTIME_MODULE is not None:
        return _RUNTIME_MODULE

    here = Path(__file__).resolve().parent
    runtime_zip = here / "cae_runtime.zip"
    if not runtime_zip.exists():
        raise FileNotFoundError("Missing cae_runtime.zip beside drone_agent.py")

    runtime_dir = Path(tempfile.gettempdir()) / (
        f"cae_runtime_{runtime_zip.stat().st_size}_{int(runtime_zip.stat().st_mtime)}"
    )
    marker = runtime_dir / ".extracted"

    if not marker.exists():
        runtime_dir.mkdir(parents=True, exist_ok=True)
        _safe_extract(runtime_zip, runtime_dir)
        marker.write_text("ok\n")

    if str(runtime_dir) not in sys.path:
        sys.path.insert(0, str(runtime_dir))

    _RUNTIME_MODULE = importlib.import_module("cae_agent_impl")
    return _RUNTIME_MODULE


class DroneFlightController:
    def __init__(self):
        impl = _load_runtime_module()
        self._delegate = impl.DroneFlightController()

    def act(self, observation):
        return self._delegate.act(observation)

    def reset(self):
        return self._delegate.reset()
