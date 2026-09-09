"""Validate decoded glTF hierarchy + animation data, not only the source math.

This catches the important mm/Z-up -> m/Y-up conversion being dropped while
replacing static child matrices with animated node TRS.
"""
from dataclasses import replace
import json
import math
import struct
import numpy as np
from scipy.spatial.transform import Rotation
from mechanism_lab.core import axis_pose
from mechanism_lab.exporters import export_animated_glb,NATIVE_TO_GLTF


def read_glb(path):
    raw=path.read_bytes();magic,version,total=struct.unpack_from('<III',raw)
    assert magic==0x46546c67 and version==2 and total==len(raw)
    length,kind=struct.unpack_from('<II',raw,12)
    assert kind==0x4e4f534a
    doc=json.loads(raw[20:20+length]);position=20+length
    n,kind=struct.unpack_from('<II',raw,position);assert kind==0x004e4942
    return doc,raw[position+8:position+8+n]


def array(doc,data,index):
    a=doc['accessors'][index];v=doc['bufferViews'][a['bufferView']]
    dims={'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4,'MAT4':16}[a['type']]
    assert a['componentType']==5126
    offset=v.get('byteOffset',0)+a.get('byteOffset',0)
    return np.frombuffer(data,dtype='<f4',count=a['count']*dims,offset=offset).reshape(a['count'],dims)


def matrix(node):
    if 'matrix' in node:return np.array(node['matrix']).reshape(4,4).T
    out=np.eye(4)
    out[:3,:3]=Rotation.from_quat(node.get('rotation',[0,0,0,1])).as_matrix()@np.diag(node.get('scale',[1,1,1]))
    out[:3,3]=node.get('translation',[0,0,0])
    return out


def check_frames(assembly,path):
    doc,data=read_glb(path)
    parents={child:parent for parent,node in enumerate(doc['nodes']) for child in node.get('children',[])}
    named={node.get('name'):i for i,node in enumerate(doc['nodes'])}
    animation=doc['animations'][0]
    mapping={}
    for channel in animation['channels']:
        sampler=animation['samplers'][channel['sampler']]
        times=array(doc,data,sampler['input'])[:,0]
        values=array(doc,data,sampler['output'])
        mapping[(channel['target']['node'],channel['target']['path'])]=(times,values)
    sampled_times=next(iter(mapping.values()))[0]
    maximum=0.
    for sample_index in sorted({0,len(sampled_times)//3,len(sampled_times)//2,len(sampled_times)-1}):
        t=float(sampled_times[sample_index]);nodes=[dict(node) for node in doc['nodes']]
        for (node_id,component),(times,values) in mapping.items():
            assert np.array_equal(times,sampled_times)
            nodes[node_id][component]=values[sample_index].tolist()
        for p in assembly.parts:
            ni=named[p.name];actual=matrix(nodes[ni])
            while ni in parents:
                ni=parents[ni];actual=matrix(nodes[ni])@actual
            expected=NATIVE_TO_GLTF@assembly.pose(p,t)
            probe=p.vertices[[0,len(p.vertices)//2,-1]]
            av=probe@actual[:3,:3].T+actual[:3,3]
            ev=probe@expected[:3,:3].T+expected[:3,3]
            maximum=max(maximum,float(np.max(abs(av-ev))))
            assert np.max(abs(av-ev))<2e-7
    return maximum


def test_exported_animation_has_real_world_units_and_pivots(flange,tmp_path):
    def pose(p,t,e):return axis_pose(t*.8,(16,8,-3),p.explode,e)
    assembly=replace(flange,motion_function=pose)
    path=tmp_path/'moving.glb'
    export_animated_glb(assembly,path,duration=2,fps=12)
    assert check_frames(assembly,path)<2e-7


def test_delivered_motor_animation(motor,root):
    path=root/'outputs/m8325s/m8325s_motion.glb'
    if not path.exists():
        import pytest;pytest.skip('Full-release artifact is not present in the smaller regeneration kit')
    assert check_frames(motor,path)<2e-7


def test_delivered_drivetrain_animation(drivetrain,root):
    path=root/'outputs/drivetrain/drivetrain_motion.glb'
    if not path.exists():
        import pytest;pytest.skip('Full-release artifact is not present in the smaller regeneration kit')
    assert check_frames(drivetrain,path)<2e-7
