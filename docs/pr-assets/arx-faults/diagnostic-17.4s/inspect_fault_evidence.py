"""Verify this evidence packet using the Python standard library only."""
from pathlib import Path
import argparse
import gzip
import hashlib
import io
import json
import tarfile


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--packet',type=Path,default=Path(__file__).resolve().parent)
    parser.add_argument('--restore-baseline',type=Path,help='Restore exact diagnostic inputs/code into a fresh checkout; never overwrite existing files')
    args=parser.parse_args();root=args.packet
    manifest=json.loads((root/'fault-evidence-manifest.json').read_text())
    for name,record in manifest['files'].items():
        raw=(root/name).read_bytes()
        assert len(raw)==record['bytes'] and sha(raw)==record['sha256'],name
    for name,record in manifest['raw_receipts'].items():
        raw=(root/name).read_bytes()
        if name.endswith('.gz'):
            assert len(raw)==record['compressed_bytes'] and sha(raw)==record['compressed_sha256'],name
            raw=gzip.decompress(raw)
        assert len(raw)==record['raw_bytes'] and sha(raw)==record['raw_sha256'],name
    cases={}
    for prefix in ('full-suite','refined-suite'):
        raw=json.loads(gzip.decompress((root/(prefix+'-summary.json.gz')).read_bytes()))
        inputs=json.loads(gzip.decompress((root/(prefix+'-inputs.json.gz')).read_bytes()))
        compact=json.loads((root/(prefix+'-summary.json')).read_text())
        del compact['evidence_derivation']
        for row in compact['results']:
            link=row.pop('reference_source_link')
            reference=dict(source_files=inputs['source_files'],task_parameters=inputs['task_parameters'])
            canonical=json.dumps(reference,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
            assert sha(canonical)==link['semantic_sha256']
            row['reference_source']=reference
        assert compact==raw,prefix
        cases[prefix]=dict(complete=raw['complete'],passed=raw['passed'],rows=len(raw['results']),
            unexpected=[row['case']['id'] for row in raw['results'] if not row['expected_outcome_observed']])
    archive=root/'baseline-reproduction.tar.gz'
    members=manifest['raw_receipts'][archive.name]['members']
    with tarfile.open(fileobj=io.BytesIO(gzip.decompress(archive.read_bytes())),mode='r:') as tar:
        assert tar.getnames()==[row['path'] for row in members]
        payload=[]
        for row in members:
            member=tar.getmember(row['path']);assert member.isfile()
            raw=tar.extractfile(member).read();assert len(raw)==row['bytes'] and sha(raw)==row['sha256']
            payload.append((row['path'],raw))
    if args.restore_baseline is not None:
        target=args.restore_baseline.resolve()
        destinations=[target/name for name,_ in payload]
        if any(not path.resolve().is_relative_to(target) or path.exists() for path in destinations):
            raise ValueError('Baseline restore requires contained paths and no existing destination files')
        for path,(_,raw) in zip(destinations,payload):
            path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
        print('Restored original inputs/code. From that checkout run: python outputs/fault-diagnosis/diagnose_policy.py --policy source --transfer 1 --descent 1')
    print(json.dumps(dict(verified_files=len(manifest['files']),source_revision=manifest['source_revision_label'],suites=cases),indent=2))


if __name__=='__main__':main()
