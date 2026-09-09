# Publication status and safe Git handoff

The requested GitHub destination was not present in the user message. Connected-repository listings and searches did not identify an explicitly designated differential repository. Consequently **this release has not been pushed to a remote repository**. No unrelated repository was chosen and no existing project was overwritten. `validation/publication.json` records the state.

The prepared README already uses relative image and GIF paths. All referenced media is part of the full release. Actual source, model inputs, geometry exports, tests, historical assets and render tools are included, rather than a README linking to temporary chat files. The full package is structured as a normal Git working tree; the optional bundle is a real locally committed Git history.

## Push only to the exact intended repository

The full release uses ordinary Git blobs. No unresolved Git LFS pointers or third-party hosting links stand in for media. The validation step checks that individual files fit below 100 MiB. The overall repository is intentionally substantial because it preserves all previous and new media/CAD. Future releases may choose a different asset policy, but this version does not silently omit the files requested for publication.

After obtaining the exact `owner/repository`, an authenticated Git environment can use the included helper:

```bash
python tools/publish.py --repo OWNER/REPOSITORY --branch mechanism-lab-v0.4.0
```

The helper operates only on an already committed local working tree. It checks for uncommitted changes, requires an explicit destination and branch, inspects the remote and refuses to update an existing remote branch. It never creates a repository, deletes remote history, force-pushes, requests credentials in the script, or selects an account based on a guessed name. An empty or existing repository can receive the newly named branch; merging an existing project's root contents is a separate owner decision.

The supplied GitHub workflow tests the toolkit and renders an independent flange sample. Its presence does not mean remote CI has run or passed. Local validation results are separately recorded in `validation/pytest.xml`.

No project-wide license was chosen without the owner's instruction. Existing notices and third-party boundaries are recorded in `docs/PROVENANCE.md`.

The downloadable ZIP deliberately excludes `.git/`. To use the publishing helper from that ZIP, first initialize a repository and commit the reviewed files with your own Git author configuration:

```bash
git init -b main
git add .
git commit -m "Publish Mechanism Lab geometry, media and reusable tools"
```

Alternatively, clone the separately provided local Git bundle. The bundle includes the committed full release history; cloning it does not itself upload anything to GitHub.
