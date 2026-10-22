"""Built-in recipes plus trusted local Python plugins (`path.py` or module:factory)."""
from __future__ import annotations
from pathlib import Path
import hashlib,importlib,importlib.util,json
from .core import Assembly,project_root,save_cache,load_cache,validate

DEFAULT_RECIPE='compact_arm'
CONSTRUCTED=('compact_arm','belt_arm','inspection_head')
BUILTINS=CONSTRUCTED+('m8325s','nitinol_fiber_actuator','nitinol_fiber_actuator_v2','differential_reference','differential_core','differential_working','drivetrain','example_flange')

def factory(name):
    if name in CONSTRUCTED:
        from .constructed import build,pose
        return lambda:build(name),pose
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
        descriptor=json.loads(Path(name).read_text(encoding='utf-8'))
        if descriptor.get('schema')=='cybrgeo-mechanism/1':
            from .constructed import build,pose
            return lambda:build(descriptor.get('architecture',DEFAULT_RECIPE),descriptor.get('parameters')),pose
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
    for package in (Path(__file__).parent,Path(__file__).parent.parent/'cybrgeo'):
        for p in sorted(package.rglob('*.py')):
            h.update(str(p.relative_to(package.parent)).encode());h.update(p.read_bytes())
    if name.endswith('.py'):
        import ast
        recipe=Path(name).resolve();raw=recipe.read_bytes();h.update(raw)
        # Trusted local recipes may declare sibling geometry/motion inputs.
        # Read literal path lists without executing the recipe a second time.
        for node in ast.parse(raw).body:
            if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='CACHE_DEPENDENCIES' for t in node.targets):
                for dependency in ast.literal_eval(node.value):
                    file=(recipe.parent/dependency).resolve();h.update(str(file).encode());h.update(file.read_bytes())
    if name.endswith('.json'):
        path=Path(name).resolve();raw=path.read_bytes();h.update(raw)
        spec=json.loads(raw)
        if spec.get('schema')=='cybrgeo-mechanism/1':return h.hexdigest()
        source=path.parent/spec['source'];h.update(source.read_bytes())
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

def cache_directory(name):
    slug=Path(name).stem.replace(':','_')
    if name not in BUILTINS:
        slug+='_'+hashlib.sha256(str(Path(name).resolve()).encode()).hexdigest()[:10]
    return project_root()/'outputs'/slug/'cache'


def load(name, rebuild=False, analytic=False):
    fn,motion=factory(name);cache=cache_directory(name);sig=fingerprint(name)
    constructed=name in CONSTRUCTED or (name.endswith('.json') and
        json.loads(Path(name).read_text(encoding='utf-8')).get('schema')=='cybrgeo-mechanism/1')
    valid=(cache/'manifest.json').exists() and (cache/'validation.json').exists() and (cache/'fingerprint.txt').exists() and (cache/'fingerprint.txt').read_text()==sig
    if valid and constructed:
        try:
            hashes=json.loads((cache/'construction-cache.json').read_text())
            valid=isinstance(hashes,dict) and set(hashes)=={'manifest.json','meshes.npz'} and all(
                hashlib.sha256((cache/file).read_bytes()).hexdigest()==digest
                for file,digest in hashes.items())
        except (OSError,ValueError,TypeError):valid=False
    if valid and not rebuild and not analytic:
        assembly=load_cache(cache)
        assembly.motion_function=motion.bind(assembly) if callable(getattr(motion,'bind',None)) else motion
        return assembly
    assembly=fn()
    if not isinstance(assembly,Assembly):raise TypeError('Recipe must return mechanism_lab.Assembly')
    report=validate(assembly);save_cache(assembly,cache)
    (cache/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if constructed:
        hashes={file:hashlib.sha256((cache/file).read_bytes()).hexdigest()
                for file in ('manifest.json','meshes.npz')}
        (cache/'construction-cache.json').write_text(json.dumps(hashes,indent=2)+'\n')
    # Write the completion marker last; interrupted builds cannot be reused.
    (cache/'fingerprint.txt').write_text(sig)
    return assembly
