These test-only JavaScript extracts come from the user's ARX belt source at
commit `aabdf8c57a4ee64f77b8a246ad9aca7b51d9cba2`. They execute the original
equations as a numerical oracle for the shared Python kernels. They are not a
second runtime, an ARX replacement, or evidence of hardware qualification.

Original directory: `ARX-6-P5-Engineering/integration/`.

| Original file | SHA-256 |
| --- | --- |
| `controller.mjs` | `6c4bf36dfc8cbc034a81eef2930d5287b1cd09650e55b2f86dae04a44e8aa9de` |
| `safety.mjs` | `10e967f2efd33ccb3ad547d18dfea57548aed2ebb7b0f4eebcc426f68eff06c5` |
| `parameters.json` | `0da3126e257dcffa653c28ec1f26d9b4aba9bffc63e564f60e98f2f4da5c168f` |
| `spec.json` | `c1df4680e4703142bb7798cf2bf6e2d8918e0dee0dde49d66a21146682f0b1ed` |

`source-safety.mjs` is byte-for-byte original. The motor and gripper blocks are
unmodified contiguous source statements beginning respectively at `const
N=par.gear_ratio[i]` and `const requestI=clip`. The current-budget block contains
the original three statements between its previous-current test and `requestI`.
`parameters.json` retains the original motor and gripper data plus the spring,
brake, soft-limit and gripper-driver specifications needed by the tests.

Source limitations are retained: ideal current reset on power loss; lumped
provisional thermal constants; screening gearbox/belt limits; brake delay and
dynamic torque assumptions; nominal spring data. The generic safety port adds
finite velocity-feedback rejection and parameterizes the source's fixed 0.35 s
controlled stop, 0.1 s watchdog and 0.002 rad travel margin. Nominal parity uses
those original values. CurrentBudget parameterizes the source's hardcoded
0.1875 A continuous-current reference; the supplied example retains that value.
