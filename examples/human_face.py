"""Default human-face recipe: original procedural anatomy, with no scan inputs."""
from pathlib import Path
import importlib.util
import sys

_path=Path(__file__).with_name('procedural_human_face.py')
_spec=importlib.util.spec_from_file_location('cybr_original_human_face',_path)
_module=importlib.util.module_from_spec(_spec)
sys.modules[_spec.name]=_module
_spec.loader.exec_module(_module)
FaceParameters=_module.FaceParameters
Anatomy=_module.Anatomy
build=_module.build
