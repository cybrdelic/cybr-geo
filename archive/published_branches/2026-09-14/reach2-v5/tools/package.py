"""Create an audited transport ZIP from the workspace, excluding rebuildable caches."""
from pathlib import Path
import argparse,hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1]
EXCLUDE={'.git','.venv','__pycache__','.pytest_cache','.cache','build','dist','.publication','.publication2'}
def entries():
 return sorted(p for p in ROOT.rglob('*') if p.is_file() and not any(s in EXCLUDE for s in p.relative_to(ROOT).parts) and p.suffix not in {'.pyc','.nbc','.nbi','.ttf','.otf'})
def main():
 p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();out=Path(a.out).resolve()
 if out.is_relative_to(ROOT):raise ValueError('Place transport ZIP outside the source tree')
 fs=[f for f in entries() if f.relative_to(ROOT)!=Path('docs/delivery_manifest.json')]
 manifest={'schema':'cybrgeo.delivery/1','files':{str(f.relative_to(ROOT)):{'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in fs}}
 (ROOT/'docs/delivery_manifest.json').write_text(json.dumps(manifest,indent=2))
 out.parent.mkdir(parents=True,exist_ok=True)
 with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED,6) as z:
  for f in entries():z.write(f,f.relative_to(ROOT))
 print(json.dumps({'file':str(out),'bytes':out.stat().st_size,'sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'files':len(manifest['files'])+1},indent=2))
if __name__=='__main__':main()
