"""Build CAD and bind the inspection deliverables to independent evidence."""
import subprocess,sys,json,hashlib
from pathlib import Path
OUT=Path(__file__).resolve().parent
def check_current():
    r=json.loads((OUT/'verification/review.json').read_text())
    if not r['geometry_review_passed']:raise ValueError('Geometry review did not pass; inspect verification/review.json')
    for name,digest in r['fingerprints'].items():
        if hashlib.file_digest((OUT/name).open('rb'),'sha256').hexdigest()!=digest:raise ValueError('Changed review input: '+name)
    return r
if __name__=='__main__':
    if '--check-existing' not in sys.argv:
        for name in ['model.py','export_inspection.py','review.py']:subprocess.run([sys.executable,str(OUT/name)],check=True)
    r=check_current();print(json.dumps({'geometry_review_passed':True,'fabrication_release':False,'parts':r['parts']}))
