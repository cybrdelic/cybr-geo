import unittest
import numpy as np
from cybrgeo import (Feature, SurfaceAnchor, Correspondence, Graph, BuildError,
                     Mesh, Attribute, extrude, parametric_surface, refine)


def curved():
    return parametric_surface(lambda u,v: np.stack([u,v,.4*u*u+.2*v*v],axis=-1),
                              np.linspace(-1,1,5),np.linspace(-1,1,5))


def pin(frame):
    a=np.linspace(0,2*np.pi,16,endpoint=False)
    m=extrude(np.stack([.05*np.cos(a),.05*np.sin(a)],axis=1),.2)
    attrs=dict(m.attributes)
    attrs['base']=Attribute((np.abs(m.vertices[:,2])<1e-10).astype(float),'vertex','nearest')
    return Mesh(m.vertices,m.faces,attrs)


class SurfaceFeatureTests(unittest.TestCase):
    def test_triangle_frame_and_offset(self):
        m=curved(); a=SurfaceAnchor.on(m,5,[.2,.3,.5])
        frame=Feature.at(a,offset=.15).resolve(m)
        t=m.vertices[m.faces[5]]
        normal=np.cross(t[1]-t[0],t[2]-t[0]);normal/=np.linalg.norm(normal)
        np.testing.assert_allclose(frame.origin,a.point(m)+.15*normal)
        np.testing.assert_allclose(frame.axis,normal)
        np.testing.assert_allclose(frame.matrix[:3,:3].T@frame.matrix[:3,:3],np.eye(3),atol=1e-12)
        self.assertIsNone(frame.radius)

    def test_stale_fails_and_explicit_deformation_follows(self):
        m=curved(); a=SurfaceAnchor.on(m,5,[.2,.3,.5]); f=Feature.at(a)
        n=Mesh(m.vertices+[2,3,4],m.faces,m.attributes,closed=False)
        with self.assertRaisesRegex(ValueError,'another snapshot'): f.resolve(n)
        followed=f.follow(Correspondence.deformation(m,n))
        np.testing.assert_allclose(followed.resolve(n).origin,f.resolve(m).origin+[2,3,4])

    def test_refinement_and_missing_lineage(self):
        m=curved(); f=Feature.at(SurfaceAnchor.on(m,5,[.2,.3,.5]))
        r=refine(m,.3)
        followed=f.follow(Correspondence.refinement(m,r))
        np.testing.assert_allclose(followed.resolve(r.mesh).origin,f.resolve(m).origin)
        c=Correspondence.from_lineage(m,r.mesh,np.full(len(r.mesh.faces),-1),
                                      np.tile(np.eye(3),(len(r.mesh.faces),1,1)))
        with self.assertRaisesRegex(ValueError,'no surviving'): f.follow(c)

    def test_graph_fit_accepts_surface_feature_and_rejects_stale_edit(self):
        m=curved(); target=Feature.at(SurfaceAnchor.on(m,5,[.2,.3,.5]),offset=.1)
        g=Graph();g.input('dz',0.)
        g.op('host',lambda dz: Mesh(m.vertices+[0,0,dz],m.faces,m.attributes,closed=False),'dz')
        g.fit('pin',pin,'host',target=target,source=Feature('base'),assume_valid_solid=True)
        good=g.build()
        fitted=Feature('base',normal_hint=tuple(target.resolve(m).axis)).resolve(good.outputs['pin'])
        np.testing.assert_allclose(fitted.origin,target.resolve(m).origin,atol=1e-10)
        with self.assertRaisesRegex(BuildError,'another snapshot'):g.edit(dz=1.)
        self.assertIs(g.last,good)

    def test_graph_feature_dependency_follows_host_edit(self):
        m=curved(); target=Feature.at(SurfaceAnchor.on(m,5,[.2,.3,.5]),offset=.1)
        g=Graph();g.input('dz',0.)
        g.op('host',lambda dz: Mesh(m.vertices+[0,0,dz],m.faces,m.attributes,closed=False),'dz')
        g.op('contact',lambda host: target.follow(Correspondence.deformation(m,host)),'host')
        g.fit('pin',pin,'host',target='contact',source=Feature('base'),assume_valid_solid=True)
        first=g.build();second=g.edit(dz=2.)
        np.testing.assert_allclose(second.outputs['pin'].vertices,first.outputs['pin'].vertices+[0,0,2],atol=1e-10)
        self.assertIn('contact',second.report['evaluated'])
        self.assertIn('pin',second.report['evaluated'])
        g.replace('contact',lambda host: None)
        with self.assertRaisesRegex(BuildError,'did not produce'):g.build()
        self.assertIs(g.last,second)

    def test_invalid_reference_and_phase_fail(self):
        m=curved();a=SurfaceAnchor.on(m,5,[.2,.3,.5]);n=Feature.at(a).resolve(m).axis
        with self.assertRaisesRegex(ValueError,'parallel'):Feature.at(a,x_hint=n).resolve(m)
        with self.assertRaises(ValueError):Feature.at(a,offset=np.nan)
        with self.assertRaises(TypeError):Feature.at({'face':1})
        for b in ([np.nan,.5,.5],[np.inf,0,0],[1,1,1],[-1,1,1]):
            with self.assertRaises(ValueError):SurfaceAnchor.from_dict(dict(snapshot=a.snapshot,face=1,barycentric=b))
        for face in (-1,1.2,True):
            with self.assertRaises(ValueError):SurfaceAnchor.from_dict(dict(snapshot=a.snapshot,face=face,barycentric=[1,0,0]))
        with self.assertRaisesRegex(ValueError,'does not exist'):
            SurfaceAnchor(a.snapshot,100000,(1,0,0)).point(m)

if __name__=='__main__':unittest.main()
