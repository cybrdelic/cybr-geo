"""Copy only successful task outputs into ordinary Git-backed delivery paths."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import zipfile

source=Path("build/vr_headset")
media=Path("media/vr_headset");media.mkdir(parents=True,exist_ok=True)
reports=Path("docs/validation/vr_headset");reports.mkdir(parents=True,exist_ok=True)
for name in ("verification.json","viewer_verification.json"):
    data=json.loads((source/name).read_text())
    if not data["passed"]:
        raise SystemExit("Refusing to publish failed validation")
for p in (source/"renders").glob("*"):
    if p.suffix in (".png",".mp4",".gif"):
        shutil.copy2(p,media/p.name)
    elif p.suffix==".json":
        shutil.copy2(p,reports/p.name)
for p in source.glob("*.json"):
    shutil.copy2(p,reports/p.name)
for name in ("visor_m1.glb","visor_m1_ipd.glb"):
    shutil.copy2(source/name,media/name)
with zipfile.ZipFile(media/"visor_m1_cad.zip","w",zipfile.ZIP_DEFLATED) as z:
    for p in sorted(source.rglob("*")):
        if p.is_file() and (p.suffix==".step" or "print_parts" in p.parts or "bom" in p.parts):
            z.write(p,p.relative_to(source))
receipt={"source_commit":os.environ.get("GITHUB_SHA"),"workflow_run":os.environ.get("GITHUB_RUN_ID"),
         "generated_imagery":False,"physical_device_tested":False,
         "files":{p.name:{"bytes":p.stat().st_size,
                            "sha256":hashlib.sha256(p.read_bytes()).hexdigest()}
                  for p in sorted(media.iterdir()) if p.is_file()}}
(reports/"receipt.json").write_text(json.dumps(receipt,indent=2)+"\n")
