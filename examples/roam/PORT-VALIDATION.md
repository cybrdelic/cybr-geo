# Port validation

Tested against CYBR GEO base `999b9baea0284b892654ef9cdeb89d1ca5049218` using the existing Windows Python CAD environment.

| Check | Result |
| --- | --- |
| Python syntax | All 22 Python files parsed |
| `python examples/roam/smoke.py` | All seven workshop recipes: valid single solids, all-pair exact nominal static interference and declared connection gaps passed |
| `python examples/roam/run.py bend` | Full checked build passed: 48 parts, 4 printed bodies, 33 movement samples, service/retention/defect gates and fingerprinted exports |
| `python examples/roam/run.py spine` | Fresh export: 642 bodies, 19 driven mates, maximum mate residual approximately 1.17e-8 mm; eight-part inspection passed; all six selected poses passed. Full sweep/service review is still in progress at this initial submission |
| Image inspection | Both reduced historical render assets visually reviewed; they remain illustrative historical outputs |

Static workshop audit counts:

| Machine | Bodies | Unordered pairs | Exact intersections |
| --- | ---: | ---: | ---: |
| table | 211 | 22155 | 292 |
| drill | 162 | 13041 | 244 |
| bend | 48 | 1128 | 67 |
| vise | 104 | 5356 | 151 |
| measure | 141 | 9870 | 197 |
| press | 144 | 10296 | 204 |
| electronics | 100 | 4950 | 131 |

The full motion/export gates for the six other workshop recipes were not rerun in this port. Their earlier workspace receipts were rechecked before packaging and are summarized separately in `historical-evidence.json`; this does not transfer acceptance to modified source. No physical prints, loads, friction, thermal tests or machine operations were performed. No new photographic render was required for this source submission.

The environment emitted an ezdxf user-cache write warning; the reported CAD checks continued successfully. Generated machine files and local logs are intentionally not committed. Reproduction regenerates receipts in the selected build directory.
