"""Render-time truth/provenance gates.

The geometry pipeline intentionally supports reference reconstruction, original
concept design, and inspection of uncertain/estimated geometry. Those are
different claims. This module prevents estimated or unknown geometry from
silently entering a reference or concept render.

The gate is deliberately based on explicit Part.provenance metadata rather than
part names, materials, visual appearance, or model-specific heuristics.
"""
from __future__ import annotations

from collections import Counter
import json
from pathlib import Path


AUTHORITATIVE={
    'measured','manufacturer-cad','manufacturer-reference','dimensioned-drawing',
    'reconstructed-from-drawings','reconstructed-from-photos','reference-reconstruction',
    'preserved-v3-geometry','imported-analytic-cad','imported-mesh','physically-derived',
}
DESIGNED={'designed-concept','source-guided-concept'}
ESTIMATE_MARKERS=('estimate','estimated','inferred','illustrative','placeholder','representative','assumed','proxy','mock','generic')
INTENTS={'reference','concept','inspection'}


def _normalise(value:str|None)->str:
    return str(value or 'unknown').strip().lower().replace('_','-').replace(' ','-')


def provenance_tier(value:str|None)->str:
    p=_normalise(value)
    if p in AUTHORITATIVE:return 'authoritative'
    if p in DESIGNED:return 'designed'
    if any(marker in p for marker in ESTIMATE_MARKERS):return 'estimated'
    return 'unknown'


def resolve_intent(assembly,intent:str='auto')->str:
    if intent=='auto':
        declared=assembly.metadata.get('truth_intent')
        if declared is not None:
            intent=str(declared).strip().lower()
        else:
            # A model composed solely of source-backed and explicitly original
            # design geometry can safely identify itself as a CONCEPT render.
            # The presence of even one estimated/unknown part instead falls back
            # to reference, where that part will be blocked rather than hidden.
            tiers={provenance_tier(part.provenance) for part in assembly.parts}
            intent='concept' if tiers and tiers <= {'authoritative','designed'} and 'designed' in tiers else 'reference'
    if intent not in INTENTS:raise ValueError(f"Unknown render truth intent {intent!r}; expected one of {sorted(INTENTS)} or 'auto'")
    return intent


def truth_report(assembly,intent:str='auto',allow_estimates:bool=False)->dict:
    """Describe whether an assembly can be rendered under the requested claim.

    reference: source-backed/authoritative geometry only.
    concept: authoritative + explicitly original/source-guided design geometry.
    inspection: all geometry visible; uncertainty remains explicit in the report.
    """
    resolved=resolve_intent(assembly,intent);allowed={'authoritative'}
    if resolved=='concept':allowed.add('designed')
    elif resolved=='inspection':allowed|={'designed','estimated','unknown'}
    if allow_estimates:allowed.add('estimated')
    rows=[]
    for part in assembly.parts:
        tier=provenance_tier(part.provenance)
        rows.append({'name':part.name,'group':part.group,'role':part.role,'provenance':part.provenance,'tier':tier,'allowed':tier in allowed})
    counts=Counter(row['tier'] for row in rows);blocked=[row for row in rows if not row['allowed']];uncertain=[row for row in rows if row['tier'] in {'estimated','unknown'}]
    return {
        'model':assembly.name,'requested_intent':intent,'resolved_intent':resolved,'allow_estimates':bool(allow_estimates),
        'passed':not blocked,'part_count':len(rows),'tier_counts':dict(sorted(counts.items())),'blocked':blocked,'uncertain':uncertain,'parts':rows,
        'claim':{
            'reference':'source-backed/reference geometry only',
            'concept':'explicitly designed concept geometry; no estimates or unknowns',
            'inspection':'inspection output may contain estimates/unknowns and must not be presented as reference geometry',
        }[resolved],
    }


def assert_renderable(assembly,intent:str='auto',allow_estimates:bool=False)->dict:
    report=truth_report(assembly,intent,allow_estimates)
    if report['passed']:return report
    offenders=report['blocked'];shown=offenders[:12]
    detail='\n'.join(f"  - {row['name']}: {row['provenance']} ({row['tier']})" for row in shown)
    if len(offenders)>len(shown):detail+=f"\n  - ... and {len(offenders)-len(shown)} more"
    raise ValueError(
        f"Truth gate blocked {len(offenders)} component(s) for {report['resolved_intent']!r} rendering.\n{detail}\n"
        "Use an explicitly truthful intent (for example --intent concept for original design work, or --intent inspection for uncertain internals). "
        "--allow-estimates permits estimated geometry but never unknown provenance."
    )


def write_truth_report(report:dict,path)->Path:
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,indent=2)+'\n');return path
