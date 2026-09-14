"""Verify the full delivery inventory without executing any archived project code."""
from pathlib import Path
import hashlib,json,sys
ROOT=Path(__file__).resolve().parents[1]
def audit(root=ROOT):
 root=Path(root);manifest=root/'docs/delivery_manifest.json'
 if not manifest.exists():raise FileNotFoundError('The full delivery inventory is not installed')
 rows=json.loads(manifest.read_text())['files'];errors=[]
 for name,entry in rows.items():
  p=root/name
  if not p.is_file():errors.append((name,'missing'));continue
  if p.stat().st_size!=entry['bytes'] or hashlib.sha256(p.read_bytes()).hexdigest()!=entry['sha256']:errors.append((name,'checksum mismatch'))
 result={'files_expected':len(rows),'files_verified':len(rows)-len(errors),'errors':errors}
 print(json.dumps(result,indent=2));return result
if __name__=='__main__':sys.exit(bool(audit()['errors']))
