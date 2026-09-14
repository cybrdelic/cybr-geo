"""Fetch the human-face input files and verify every recorded SHA-256 digest."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT/'assets/portrait'


def fetch(record):
    target = ASSETS/record['file']
    expected = record['sha256']
    if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() == expected:
        return target.name + ': verified cached input'
    temporary = target.with_name(target.name + '.download')
    request = urllib.request.Request(record['source'], headers={'User-Agent':'CYBR-GEO-portrait/1.0'})
    with urllib.request.urlopen(request, timeout=60) as response:
        temporary.write_bytes(response.read())
    digest = hashlib.sha256(temporary.read_bytes()).hexdigest()
    if digest != expected:
        temporary.unlink(missing_ok=True)
        raise RuntimeError(f'Input checksum mismatch: {target.name}. Upstream may have changed.')
    temporary.replace(target)
    return target.name + ': downloaded and verified'


def main():
    records = json.loads((ASSETS/'sources.json').read_text())
    with ThreadPoolExecutor(max_workers=4) as pool:
        for result in pool.map(fetch, records):
            print(result)


if __name__ == '__main__':
    main()
