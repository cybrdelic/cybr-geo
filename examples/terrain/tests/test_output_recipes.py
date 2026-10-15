"""Receipts must remain reproducible after another view or detail recipe."""
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile

from cybr_terrain import cli
from cybr_terrain.model import Config, make_terrain, State
from cybr_terrain import rendering


def test_views_with_different_geometry_keep_earlier_assets_and_receipts(tmp_path,monkeypatch):
    state=make_terrain(Config(grid=17,duration=0));state.save(tmp_path/'state.npz')
    def geometry(state,directory,subdivision,stones):
        directory=Path(directory);texture=directory/'textures'/'soil.pfm'
        texture.parent.mkdir();texture.write_bytes(f'material:{subdivision}:{stones}'.encode())
        class FakeGeometry:
            atlases={0:SimpleNamespace(color_pfm=texture,normal_pfm=texture,roughness_pfm=texture)}
            def export_glb(self,path):Path(path).write_bytes(f'geometry:{subdivision}:{stones}'.encode());return path
            def validate(self):return {'passed':True}
        return FakeGeometry()
    def daylight(scene,directory):
        path=Path(directory)/'daylight.pfm'
        if not path.exists():path.write_bytes(b'stable sky')
        return {}
    def render(geo,image,quality,threads,width,spp,view):
        image=Path(image);image.write_bytes(view.encode())
        image.with_suffix('.cys').write_text(str(geo.atlases[0].color_pfm.resolve()))
        image.with_suffix('.clm').write_bytes(b'mesh')
        image.with_suffix('.json').write_text('{"passed":true}')
        return {'renderer':'fake native','width':32,'height':24,'packets_per_pixel':2,'wavelengths_per_packet':4}
    monkeypatch.setattr(cli,'build_geometry',geometry)
    monkeypatch.setattr(cli,'render_terrain',render)
    monkeypatch.setattr(rendering,'daylight',daylight)
    args=SimpleNamespace(subdivision=1,stones=1,view='hero',quality='smoke',threads=1,width=32,spp=2,plots=False)
    cli.output(state,tmp_path,args)
    hero=json.loads((tmp_path/'hero-receipt.json').read_text())
    retained={name:(tmp_path/name).read_bytes() for name in hero['files']}
    original_assets=tmp_path/hero['asset_directory']
    mtimes={p.relative_to(original_assets):p.stat().st_mtime_ns for p in original_assets.rglob('*') if p.is_file()}
    args.view='macro';args.stones=2;cli.output(state,tmp_path,args)
    macro=json.loads((tmp_path/'macro-receipt.json').read_text())
    assert macro['asset_directory']!=hero['asset_directory']
    assert (tmp_path/'terrain.glb').read_bytes()==b'geometry:1:2'
    for name,previous in retained.items():
        assert (tmp_path/name).read_bytes()==previous
        assert cli.sha(tmp_path/name)==hero['files'][name]
    assert not any(Path(name).name.startswith('hero') for name in macro['files'])
    assert 'terrain.glb' not in hero['files'] and 'geometry.json' not in hero['files']
    assert str(original_assets/'textures'/'soil.pfm') in (tmp_path/'hero.cys').read_text()
    args.view='overhead';args.stones=1;cli.output(state,tmp_path,args)
    assert mtimes=={p.relative_to(original_assets):p.stat().st_mtime_ns for p in original_assets.rglob('*') if p.is_file()}


def test_matched_but_unreadable_cache_regenerates_the_storm(tmp_path):
    args=SimpleNamespace(grid=17,seed=20260930,duration=0,rain=0,inlet=0,
                         morphological_factor=None,output=tmp_path,force=False)
    _,out=cli.simulate(args,'soil-profile')
    archive=out/'state.npz';archive.write_bytes(archive.read_bytes()[:-80])
    receipt=out/'simulation.json';old=json.loads(receipt.read_text())
    old['state_sha256']=cli.sha(archive);receipt.write_text(json.dumps(old))
    restored,_=cli.simulate(args,'soil-profile')
    assert restored.validate()
    with zipfile.ZipFile(archive) as data:assert data.testzip() is None
    assert State.load(archive).validate()
