"""Exact, guarded Blender volume absorption + scattering coefficient lowering.

Does not change the surface shader, mesh, UVs, textures, density units or phase.
Only static unlinked inputs of an unmixed two-node Add volume are supported.
Cycles counts each legacy volume node at MAX_VOLUME_STACK_SIZE (32); fusing
its coefficients avoids the 66>64 conservative allocation with a glass surface.
This is not evidence that the old rendering dropped closures or had wrong color.
"""
import math


def coefficients(absorption_color, absorption_density, scatter_color, scatter_density):
    values = (*absorption_color[:3], absorption_density, *scatter_color[:3], scatter_density)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite optical coefficient input')
    if any(v < 0 or v > 1 for v in (*absorption_color[:3], *scatter_color[:3])):
        raise ValueError('Only unit-range colors are supported')
    return (tuple((1-c)*max(0, absorption_density) for c in absorption_color[:3]),
            tuple(c*max(0, scatter_density) for c in scatter_color[:3]))


def lower_material(material):
    """Return a receipt; reject unsupported graphs before mutation."""
    t=material.node_tree
    if t is None:return None
    outs=[n for n in t.nodes if n.type=='OUTPUT_MATERIAL' and n.is_active_output]
    if len(outs)!=1:return None
    out=outs[0];vl=out.inputs['Volume'].links
    if len(vl)!=1:return None
    add=vl[0].from_node
    if add.bl_idname!='ShaderNodeAddShader':return None
    branches=[i.links[0].from_node for i in add.inputs if len(i.links)==1]
    if len(branches)!=2:return None
    bytype={n.bl_idname:n for n in branches}
    if set(bytype)!={'ShaderNodeVolumeAbsorption','ShaderNodeVolumeScatter'}:return None
    a=bytype['ShaderNodeVolumeAbsorption'];s=bytype['ShaderNodeVolumeScatter']
    if any(i.is_linked for n in (a,s) for i in n.inputs):
        raise ValueError(f'{material.name}: dynamic inputs not supported')
    # Avoid modifying shared branches or silently changing another output.
    if any(len(n.outputs[0].links)!=1 for n in (a,s,add)):
        raise ValueError(f'{material.name}: shared volume branches not supported')
    sigma_a,sigma_s=coefficients(a.inputs['Color'].default_value,a.inputs['Density'].default_value,
                                s.inputs['Color'].default_value,s.inputs['Density'].default_value)
    phase=getattr(s,'phase','HENYEY_GREENSTEIN')
    n=t.nodes.new('ShaderNodeVolumeCoefficients')
    try:
        if hasattr(n,'phase'):n.phase=phase
        for key in ('Anisotropy','IOR','Backscatter','Alpha','Diameter'):
            if key in s.inputs and key in n.inputs:n.inputs[key].default_value=s.inputs[key].default_value
        n.inputs['Absorption Coefficients'].default_value=sigma_a
        n.inputs['Scatter Coefficients'].default_value=sigma_s
        n.inputs['Emission Coefficients'].default_value=(0,0,0)
    except Exception:
        t.nodes.remove(n)
        raise
    n.name='Exact PETG volume coefficients';n.label='sigma_a + sigma_s; original phase, units and surface'
    n.location=add.location
    t.links.new(n.outputs[0],out.inputs['Volume'])
    for old in (a,s,add):t.nodes.remove(old)
    return {'material':material.name,'sigma_a':sigma_a,'sigma_s':sigma_s,'phase':phase,
            'surface_unchanged':True,'closure_estimate_before':66,'closure_estimate_after':34}
