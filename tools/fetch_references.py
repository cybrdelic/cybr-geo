"""Download only the recorded public manufacturer references; never infer CAD provenance."""
from pathlib import Path
import hashlib,json,urllib.request
ROOT=Path(__file__).resolve().parents[1]
FILES={
 'NEO_Vortex_SPARK_Flex_8mm.STEP':'https://www.revrobotics.com/content/cad/NEO-Vortex-Moter-and-SPARK-Flex-Motor-Controller-with-8mm-Shaft.STEP',
 'REV-21-1652-REV-11-2159-DR.pdf':'https://www.revrobotics.com/content/docs/REV-21-1652-REV-11-2159-DR.pdf'}
def main():
 root=ROOT/'references/neo_vortex';root.mkdir(parents=True,exist_ok=True);receipts={}
 for name,url in FILES.items():
  target=root/name
  if not target.exists():
   with urllib.request.urlopen(url,timeout=120) as response:data=response.read()
   if not data:raise RuntimeError('Empty manufacturer response')
   tmp=target.with_suffix(target.suffix+'.partial');tmp.write_bytes(data);tmp.replace(target)
  data=target.read_bytes();receipts[name]={'url':url,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
 (root/'download_receipts.json').write_text(json.dumps(receipts,indent=2))
 print(json.dumps(receipts,indent=2))
if __name__=='__main__':main()
