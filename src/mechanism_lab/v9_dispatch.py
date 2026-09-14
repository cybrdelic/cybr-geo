"""Runtime dispatch for the true V9 renderer.

The approved ORBIT V9 artifact used Mitsuba's ``llvm_ad_rgb`` variant and keeps
that exact path. Very large assemblies can overflow/segfault Dr.Jit's LLVM scene
specialization while Mitsuba constructs hundreds of thousands of primitives.
Those scenes already use V9's lossless material/variation batching and are
rendered with Mitsuba's ``scalar_rgb`` variant instead. This changes execution
strategy, not geometry, BSDFs, lighting, sampling, camera, AOVs, OIDN, or color.
"""
from __future__ import annotations

from . import v9 as _core

LARGE_ASSEMBLY_PART_LIMIT = _core.V9_EXACT_PART_SHAPE_LIMIT


def _scalar_mitsuba():
    try:
        import mitsuba as mi
    except ImportError as error:
        raise RuntimeError(
            "V9 requires Mitsuba 3. Install cybr-geo with its current dependencies."
        ) from error
    variants=mi.variants()
    if 'scalar_rgb' not in variants:
        raise RuntimeError(f'Mitsuba scalar_rgb is unavailable; variants={variants}')
    try:
        mi.set_variant('scalar_rgb')
    except Exception:
        if mi.variant()!='scalar_rgb':
            raise
    return mi,'scalar_rgb'


def _dispatch(function,assembly,*args,**kwargs):
    if len(assembly.parts)<=LARGE_ASSEMBLY_PART_LIMIT:
        return function(assembly,*args,**kwargs)
    original=_core._mitsuba
    _core._mitsuba=_scalar_mitsuba
    try:
        result=function(assembly,*args,**kwargs)
        # Record why this otherwise identical V9 execution used scalar Mitsuba.
        if isinstance(result,dict):
            result['variant_dispatch']='scalar_rgb for large assembly stability; V9 scene/material/lighting/post contract unchanged'
        return result
    finally:
        _core._mitsuba=original


def render_v9(assembly,*args,**kwargs):
    return _dispatch(_core.render_v9,assembly,*args,**kwargs)


def render_v9_video(assembly,*args,**kwargs):
    return _dispatch(_core.render_v9_video,assembly,*args,**kwargs)
