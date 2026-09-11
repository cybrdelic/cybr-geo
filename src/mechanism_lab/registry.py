"""Built-in recipes plus trusted local Python plugins (`path.py` or module:factory)."""
from __future__ import annotations
from pathlib import Path
import hashlib,importlib,importlib.util,json
from .core import Assembly,project_root,save_cache,load_cache,validate

BUILTINS=('m8325s','nitinol_fiber_actuator','nitinol_fiber_actuator_v2','differential_reference','differential_core','differential_working','drivetrain','example_flange')

def factory(name):
    if name=='m8325s':
        from .models.motor import build,motor_pose
        return build,motor_pose
    if name=='nitinol_fiber_actuator':
        from .models.nitinol_actuator import build,pose
        return build,pose
    if name=='nitinol_fiber_actuator_v2':
        from .models.nitinol_actuator_v2 import build
        return build,None
    if name.startswith('differential_'):
        from .models.differential import build,pose
        kind=name.split('_',1)[1]
        return lambda:build(kind),pose if kind!='reference' else None
    if name=='drivetrain':
        from .models.drivetrain import build,pose
        return build,pose
    if name=='example_flange':
        from .models.example import build
        return build,None
    if name.endswith('.json'):
        from .importers import build_descriptor
        return lambda:build_descriptor(name),None
    if name.endswith('.py'):
        path=Path(name).resolve();spec=importlib.util.spec_from_file_location('mechanism_plugin_'+hashlib.sha256(str(path).encode()).hexdigest()[:12],path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module.build,getattr(module,'pose',None)
    if ':' in name:
        mod,fn=name.split(':',1);module=importlib.import_module(mod)
        return getattr(module,fn),getattr(module,'pose',None)
    raise ValueError(f'Unknown recipe {name}; choose {BUILTINS} or supply a trusted local .py plugin')

def fingerprint(name):
    root=project_root();h=hashlib.sha256(name.encode())
    for p in sorted((root/'src/mechanism_lab').rglob('*.py')):
        h.update(str(p.relative_to(root)).encode());h.update(p.read_bytes())
    if name.endswith('.py'):h.update(Path(name).read_bytes())
    if name.endswith('.json'):
        path=Path(name).resolve();raw=path.read_bytes();h.update(raw)
        spec=json.loads(raw);source=path.parent/spec['source'];h.update(source.read_bytes())
        # External glTF buffers/images also affect geometry/provenance invalidation.
        if source.suffix.lower()=='.gltf':
            gltf=json.loads(source.read_text())
            for record in gltf.get('buffers',[])+gltf.get('images',[]):
                uri=record.get('uri','')
                if uri and not uri.startswith('data:'):
                    h.update((source.parent/uri).read_bytes())
    # Preserved reference inputs also participate in invalidation, not just code.
    if 'differential' in name or name=='drivetrain':
        for p in sorted((root/'assets/differential_v3/geometry').glob('*parts.*')):h.update(p.read_bytes())
    return h.hexdigest()

def load(name, rebuild=False, analytic=False):
    fn,motion=factory(name);slug=Path(name).stem.replace(':','_')
    if name not in BUILTINS:
        slug+='_'+hashlib.sha256(str(Path(name).resolve()).encode()).hexdigest()[:10]
    root=project_root();out=root/'outputs'/slug;cache=out/'cache';sig=fingerprint(name)
    valid=(cache/'manifest.json').exists() and (cache/'fingerprint.txt').exists() and (cache/'fingerprint.txt').read_text()==sig
    if valid and not rebuild and not analytic:
        assembly=load_cache(cache);assembly.motion_function=motion;return assembly
    assembly=fn()
    if not isinstance(assembly,Assembly):raise TypeError('Recipe must return mechanism_lab.Assembly')
    validate(assembly);save_cache(assembly,cache);(cache/'fingerprint.txt').write_text(sig)
    return assembly
