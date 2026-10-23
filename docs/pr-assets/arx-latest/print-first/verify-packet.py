"""Verify this compact packet; optional source-root also checks original copies."""
import argparse
import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path)
    parser.add_argument('--write-report', action='store_true')
    args = parser.parse_args()
    packet = Path(__file__).resolve().parent
    manifest_path = packet/'SHA256SUMS.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    copied = 0
    total = 0
    for name, expected in manifest['files'].items():
        path = (packet/name).resolve()
        path.relative_to(packet)
        payload = path.read_bytes()
        assert len(payload) == expected['bytes'], name+' size mismatch'
        assert hashlib.sha256(payload).hexdigest() == expected['sha256'], name+' hash mismatch'
        total += len(payload)
        if args.source_root and 'copied_from' in expected:
            original = (args.source_root/expected['copied_from']).resolve()
            original.relative_to(args.source_root.resolve())
            assert payload == original.read_bytes(), name+' copied bytes differ'
            copied += 1
    receipt = json.loads((packet/'native-build.json').read_text(encoding='utf-8'))
    included_references = 0
    for name, expected in receipt['files_sha256'].items():
        if name in manifest['files']:
            assert manifest['files'][name]['sha256'] == expected, name+' receipt mismatch'
            included_references += 1
    suite = ElementTree.parse(packet/'parity.junit.xml').getroot().find('testsuite')
    tests = json.loads((packet/'test-run.json').read_text(encoding='utf-8'))
    assert int(suite.attrib['tests']) == tests['tests']
    assert all(suite.attrib[key] == '0' for key in ('failures', 'errors', 'skipped'))
    readback = json.loads((packet/'independent-readback.json').read_text(encoding='utf-8'))
    assert readback['passed']
    assert readback['native_build_file_sha256'] == manifest['files']['native-build.json']['sha256']
    canonical = hashlib.sha256(json.dumps(receipt, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    assert readback['native_build_canonical_sha256'] == canonical
    assert readback['construction_code_sha256'] == receipt['provenance']['construction_code_sha256']
    assert manifest['files']['source-default-spec.json']['sha256'] == receipt['provenance']['input_spec_file_sha256']
    preview = json.loads((packet/'print-first-preview-evidence.json').read_text(encoding='utf-8'))
    assert preview['image_sha256'] == manifest['files']['print-first-preview.png']['sha256']
    assert preview['native_build_sha256'] == manifest['files']['native-build.json']['sha256']
    assert preview['construction_code_sha256'] == receipt['provenance']['construction_code_sha256']
    result = dict(schema='cybrgeo-evidence-packet-verification/1', passed=True,
        manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        verified_files=len(manifest['files']), verified_bytes=total, copied_files_byte_equal=copied,
        included_native_receipt_references_verified=included_references,
        native_cad_stl_bundled=False, tests=int(suite.attrib['tests']),
        scope='Independent packet hash and copied-byte readback; native qualification scope remains unchanged')
    if args.write_report:
        (packet/'packet-verification.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
