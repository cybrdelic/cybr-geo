"""The usual face entry point must select original procedural anatomy."""
import importlib.util
from pathlib import Path

def test_default_face_has_no_scan_inputs():
    path=Path(__file__).resolve().parents[1]/'examples/human_face.py'
    spec=importlib.util.spec_from_file_location('default_face_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assembly=module.build(quality='preview',hair=False)
    assert assembly.metadata['scan_used'] is False
    assert assembly.metadata['input_assets']==[]
    assert all(p.provenance=='original-procedural' for p in assembly.parts)
