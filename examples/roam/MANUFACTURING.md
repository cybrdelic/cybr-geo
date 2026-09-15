# Can this workshop manufacture the whole workstation?

**No, not yet.** The seven prototypes do not cover the complete process chain. This map uses the preserved shared-spine design; the later DIY-metal design changes stock and adjustment methods and is not silently substituted here.

| Workstation part family | Required operations | Current coverage | Missing capability / practical route |
| --- | --- | --- | --- |
| Shared spine, telescoping base tubes and arm tubes | Stock selection, straight square cuts, hole patterns, deburring | Measuring fixture is nominal; no cutting station | Buy suitable stock; develop a supported cutoff fence/length stop around a selected metal-rated saw; verify squareness on coupons |
| Link end tongues, root blocks, slotted shelves, pitch cheeks | Plate profiling, slots, pockets, coaxial bores, datum control | X–Y stage has motion only | Outsource machining initially, or use a real mill with rated workholding; printed positioning parts do not establish metal-cutting capacity |
| Bushings, spacers, shoulders and compression sleeves | Diameter/length control, facing, reaming and fit inspection | No qualified turning or reaming setup | Purchase dimensioned standard parts where compatible; custom sleeves require a lathe or supplier service |
| Matched pivot holes and bearing seats | Drill, finish-ream coaxially, inspect alignment and fits | Manual spindle prototype cannot claim this process | Selected drill press, steel drill bushings, clamped paired-part jig, reamers and actual gauges; test joint coupon first |
| Pitch/roll friction locks | Flat parallel faces, friction material preparation, retained preload | Vise/press geometry does not qualify holding torque | Source friction material; machine mating faces; instrument slip-torque and sustained-load tests before device installation |
| Tray panel, webs and crossbeams | Panel cutting, edge finishing, drilling, attachment | No complete panel station | Saw/router with documented stock limits and workholding; PETG templates may guide setup |
| Caster adapters and base assembly | Match chosen caster mount, hole access, bolted joints | Caster remains an unselected envelope | Select actual caster data before final hole template; torque-access and static stability review |
| Linear guides, gas assists, fasteners | Select, buy, fit and retain | Nominal component envelopes only | Purchase these components; no attempt to print bearings, gas springs or structural screws |
| Cables and strain relief | Cut/fit sleeves, route slack, edge protection, connector access | Cable routing study and PCB fixture only | Print clips/covers, buy suitable cables and electrical assemblies; test routes through usable motion |
| Final assembly and verification | Alignment, fastener preload, fit, slip, deflection, stability | Specific CAD assembly/service probes | Build fixtures plus measuring equipment, torque tools and staged dummy loads; CAD cannot replace physical measurements |

## Next tool designs tied to real parts

1. **Tube cutoff setup:** metal-rated purchased saw/cutter, long stock support, adjustable length stop and replaceable sacrificial fence. Printed parts may locate/support away from heat and cutting contact. Actual tool geometry and mounting data are required before detail CAD.
2. **Paired pivot drilling fixture:** two datum faces, positive end stop, replaceable steel guide bushings and through-clamping of matched plates. Specify drill-to-ream allowance from the chosen bearing fit; print a fit coupon before a full fixture.
3. **Sleeve length and deburring fixture:** restrained stock, repeatable stop and access for purchased cutting/deburring tools. Inspect ends and lengths; do not represent it as producing precision diameters.
4. **Joint qualification stand:** rigid purchased/metal support, representative pivot coupon, measured lever arm and independently measured applied force. Establish the required holding torque and allowable drift from actual device loads before testing. No proposed load rating is implied here.
5. **Assembly datum fixture:** holds the base and mast square while bolts remain accessible; gauges tray/monitor adapters relative to the arm interfaces. Larger fixtures need joints compatible with the A1 build volume.

These are scoped requirements, **not completed tool models**. Full manufacturing still requires purchased machine/tool capability or outsourced operations. To finish their geometry, choose the actual drill/saw and stock/interfaces first; the user has only confirmed a Bambu A1 and transparent PETG, not ownership of a drill press, saw, lathe or mill.

The printable workshop is therefore a way to make useful jigs, fixtures and mechanisms around real tooling. It does not currently establish an all-printed manufacturing chain or a sub-$100 route to the workstation.
