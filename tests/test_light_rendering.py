"""Real engine input/output checks and indexed meshlet boundary failures."""
from dataclasses import replace
import json
from pathlib import Path
import struct
import subprocess
import numpy as np
import pytest
from cybr_light import Meshlets,Scene,compile_renderer,read_pfm
from mechanism_lab.light import render_light,render_light_video,write_geometry
from mechanism_lab.cli import parser
from mechanism_lab.media import Shot


def unpack(path):
    triangles=[];limits=[]
    with path.open('rb') as stream:
        magic,count,packets=struct.unpack('<4sQI',stream.read(16));assert magic==b'CLM1'
        dtype=np.dtype([('indices','u1',(3,)),('material','<u4'),('object','<u4')])
        for _ in range(packets):
            nv,nt=struct.unpack('<HH',stream.read(4));limits.append((nv,nt))
            vertices=np.frombuffer(stream.read(nv*44),'<f4').reshape(nv,11)
            records=np.frombuffer(stream.read(nt*11),dtype=dtype)
            triangles.extend((vertices[record['indices']],int(record['material']),int(record['object'])) for record in records)
        assert not stream.read()
    assert len(triangles)==count
    return triangles,limits


def test_meshlets_preserve_hard_edges_uv_seams_tint_and_component_ids(tmp_path):
    a=np.zeros((400,36),'<f4');a[:,:9]=[0,0,0,1,0,0,0,1,0];a[:,9:18]=[0,0,1]*3
    a[:,18]=np.arange(400)%3;a[:,19]=np.arange(400);a[:,26:35]=.7
    a[1,9:18]=[0,1,0]*3;a[2,20:26]=[.1,.2,.3,.4,.5,.6];a[3,26:35]=.3
    path=tmp_path/'mesh.clm'
    with Meshlets(path) as writer:writer.add(a)
    loaded,limits=unpack(path)
    assert all(nv<=64 and nt<=124 for nv,nt in limits)
    assert path.stat().st_size<len(a)*144*.25
    for index,(vertices,material,component) in enumerate(loaded):
        assert material==a[index,18] and component==index
        assert np.array_equal(vertices[:,:3],a[index,:9].reshape(3,3))
        assert np.array_equal(vertices[:,3:6],a[index,9:18].reshape(3,3))
        assert np.array_equal(vertices[:,6:8],a[index,20:26].reshape(3,2))
        assert np.array_equal(vertices[:,8:11],a[index,26:35].reshape(3,3))


def test_actual_engine_rejects_truncation_bad_indices_and_trailing_data(flange,tmp_path):
    mesh=tmp_path/'mesh.clm';write_geometry(flange,mesh)
    original=mesh.read_bytes();exe,_=compile_renderer()
    scene=Scene();scene.settings(64,64,1,4,1)
    for _ in flange.materials:scene.material()
    scene.camera((1,1,1),(0,0,0));scene.mesh(mesh);path=scene.save(tmp_path/'test.cys')
    # First triangle begins after the first meshlet's vertex array.
    nv=struct.unpack_from('<H',original,16)[0];bad=bytearray(original);bad[20+nv*44]=255
    for payload in (original[:-1],original+b'X',bad):
        mesh.write_bytes(payload)
        result=subprocess.run([str(exe),'--scene',str(path),'--out',str(tmp_path/'bad')],capture_output=True,text=True)
        assert result.returncode and 'ERROR:' in result.stderr
        assert not (tmp_path/'bad.json').exists()


def test_failed_pack_publication_removes_spool_and_preserves_old_file(tmp_path,monkeypatch):
    import cybr_light.runtime as runtime
    path=tmp_path/'mesh.clm';path.write_bytes(b'previous checked input')
    a=np.zeros((1,36),'<f4');a[0,:9]=[0,0,0,1,0,0,0,1,0];a[0,26:35]=1
    writer=Meshlets(path)
    original_replace=runtime.os.replace
    def cross_device(source,destination):
        if Path(source)==writer.temporary:
            import errno
            raise OSError(errno.EXDEV,'Simulated cross-device output mount')
        return original_replace(source,destination)
    def fail(*args):raise OSError('Simulated full disk')
    monkeypatch.setattr(runtime.os,'replace',cross_device)
    monkeypatch.setattr(runtime.shutil,'copyfile',fail)
    with pytest.raises(OSError,match='full disk'):
        with writer:writer.add(a)
    assert path.read_bytes()==b'previous checked input'
    assert not writer.temporary.exists() and not list(tmp_path.glob('*.partial'))


def test_pose_and_mm_conversion_reach_the_actual_meshlet_vertices(flange,tmp_path):
    posed=replace(flange,motion_function=lambda p,t,e:np.array([[1,0,0,100],[0,1,0,0],[0,0,1,0],[0,0,0,1]],float))
    mesh=tmp_path/'posed.clm';write_geometry(posed,mesh);triangles,_=unpack(mesh)
    original=flange.parts[0];expected=(original.vertices[original.faces[0]]+[100,0,0])*.001
    assert np.allclose(triangles[0][0][:,:3],expected)


def test_real_still_and_film_use_cybr_light(flange,tmp_path):
    image=tmp_path/'still.png'
    report=render_light(flange,image,size=(64,64),spp=8,threads=2,depth=6)
    assert report['renderer']=='CYBR LIGHT 0.2' and report['invalid_path_samples']==0
    assert report['image_generation_used'] is False and report['raw_film_preserved']
    assert report['meshlets']>0 and report['indexed_vertices']>0
    assert report['primitive_storage_bytes']<=64*report['primitives']
    assert json.loads(image.with_suffix('.json').read_text())==report
    assert report['geometry'] and report['engine_source_sha256']
    film=read_pfm(image.with_suffix('.pfm'));assert film.shape==(64,64,3) and np.isfinite(film).all()
    assert image.with_name('still_unfiltered.png').exists() and image.with_suffix('.exr').exists()
    video=tmp_path/'film.mp4'
    result=render_light_video(flange,video,[Shot('hero',1,'orbit')],(64,64),2,8,2,6,shutter_samples=2)
    assert result['frames']==2 and result['renderer']=='CYBR LIGHT 0.2'


def test_public_defaults_and_no_external_renderer_dependency():
    for command in ('render','video','film'):assert parser().parse_args([command,'model.py']).renderer=='light'
    root=Path(__file__).resolve().parents[1]
    assert 'mitsuba' not in (root/'pyproject.toml').read_text().lower()
    assert 'mitsuba' not in (root/'requirements-tested.txt').read_text().lower()
    assert not (root/'src/mechanism_lab/v9.py').exists()
