# Source publication inventory

Published ordinary source files preserve both `src/cybrgeo` and `src/mechanism_lab`, the three differential source generations, both motor/drive recipes, C++ path tracing, PBR rendering, animation/export, whiteprint tools, tests and reproduction documentation.

Three compact differential/drive previews from the original source archive are included in the README. Full historical videos, per-part CAD/render catalogues, generated motor geometry and original differential NPZ inputs are separate binary assets and have not all been transferred into this GitHub source checkout. Historical local-delivery documentation is retained for provenance; it is not evidence that all those assets have been published.

`tools/rebuild_differential_inputs.py` can regenerate the differential input arrays from the retained geometry source. It retains existing complete input arrays unless `--force` is explicit and records the regenerated origin. This is not a claim of byte-identical restoration of historical output archives. Public REV mechanical references can be fetched using the recorded source downloader.

`docs/publication_receipts` records server-side archive SHA-256 verification. Current tests or CI reports should be consulted separately from historical reports. No GitHub Actions scaffolding, local ZIP, or receipt for source alone is described as publication of the complete binary archive.
