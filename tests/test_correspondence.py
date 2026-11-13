import numpy as np
import unittest
from cybrgeo import (Mesh, SurfaceAnchor, Correspondence, CorrespondenceError,
                     project_reference, refine, reproject_anchor)


def sheet(z=0):
    return Mesh(np.array([[0,0,z],[1,0,z],[1,1,z],[0,1,z]],float),
                np.array([[0,1,2],[0,2,3]]),closed=False)


def stack(a,b):
    return Mesh(np.vstack((a.vertices,b.vertices)),np.vstack((a.faces,b.faces+4)),closed=False,connected=False)


class CorrespondenceTests(unittest.TestCase):
    def test_nearer_parallel_sheet_is_not_silent_identity(self):
        src=sheet(); target=stack(sheet(.01),sheet(.03))
        a=SurfaceAnchor.on(src,0,[.2,.3,.5])
        with self.assertRaisesRegex(CorrespondenceError,'Ambiguous'):
            project_reference(a,src,target,.1)
        with self.assertRaisesRegex(CorrespondenceError,'Ambiguous'):
            reproject_anchor(a,src,target,.1)
        r=project_reference(a,src,target,.1,source_regions=['wall']*2,
                            target_regions=['insert']*2+['wall']*2)
        np.testing.assert_allclose(r.anchor.point(target),[.8,.5,.03])
        assert r.confidence=='conditional-projection'
        assert r.anchor.face>=2


    def test_shared_triangle_edge_is_not_ambiguity(self):
        src=sheet(); a=SurfaceAnchor.on(src,0,[.5,0,.5])
        r=project_reference(a,src,sheet(.02),.1)
        self.assertAlmostEqual(r.distance,.02)


    def test_exact_refinement_then_deformation_history(self):
        src=sheet(); a=SurfaceAnchor.on(src,0,[.2,.3,.5]); ref=refine(src,.8)
        r=Correspondence.refinement(src,ref).follow(a)
        v=ref.mesh.vertices.copy(); v[:,2]=v[:,0]*.2
        deformed=Mesh(v,ref.mesh.faces,closed=False)
        r=Correspondence.deformation(ref.mesh,deformed).follow(r)
        assert len(r.lineage)==2 and r.confidence=='exact-lineage'
        np.testing.assert_allclose(r.anchor.point(deformed),[.8,.5,.16])
        assert r.to_dict()['anchor']['snapshot']==r.anchor.snapshot


    def test_stale_missing_and_branched_lineage_fail(self):
        src=sheet(); a=SurfaceAnchor.on(src,0,[.2,.3,.5])
        mapping=Correspondence.deformation(src,sheet(1))
        with self.assertRaises(CorrespondenceError): mapping.follow(SurfaceAnchor.on(sheet(.2),0,[1,0,0]))
        empty=Correspondence.from_lineage(src,src,[-1,-1],np.tile(np.eye(3),(2,1,1)))
        with self.assertRaisesRegex(CorrespondenceError,'surviving'): empty.follow(a)
        target=stack(sheet(),sheet(.1))
        branched=Correspondence.from_lineage(src,target,[0,1,0,1],np.tile(np.eye(3),(4,1,1)))
        with self.assertRaisesRegex(CorrespondenceError,'branches'): branched.follow(a)


    def test_orientation_distance_and_bad_policies(self):
        src=sheet(); a=SurfaceAnchor.on(src,0,[.2,.3,.5])
        with self.assertRaises(CorrespondenceError): project_reference(a,src,sheet(.2),.1)
        flipped=Mesh(src.vertices,src.faces[:,::-1],closed=False)
        with self.assertRaises(CorrespondenceError): project_reference(a,src,flipped,.1)
        with self.assertRaises(ValueError): project_reference(a,src,src,.1,source_regions=[0,0])
        with self.assertRaises(ValueError): Correspondence.from_lineage(src,src,[0,1],np.zeros((2,3,3)))
        with self.assertRaises(ValueError): Correspondence.deformation(src,flipped)


    def test_reordered_faces_explicit_adapter(self):
        src=sheet(); target=Mesh(src.vertices,src.faces[::-1],closed=False); a=SurfaceAnchor.on(src,0,[.2,.3,.5])
        c=Correspondence.from_lineage(src,target,[1,0],np.tile(np.eye(3),(2,1,1)),operation='CAD-face-map')
        r=c.follow(a)
        assert r.anchor.face==1
        np.testing.assert_allclose(a.point(src),r.anchor.point(target))

    def test_projection_confidence_is_not_promoted_by_exact_edit(self):
        src=sheet(); target=sheet(.01); a=SurfaceAnchor.on(src,0,[.2,.3,.5])
        guessed=project_reference(a,src,target,.1)
        result=Correspondence.deformation(target,sheet(.02)).follow(guessed)
        assert result.confidence=='conditional-projection'
        assert len(result.lineage)==2

    def test_connected_fold_does_not_merge_distant_candidate_fans(self):
        src=sheet(); v=np.array([[0,0,.01],[1,0,.01],[1,1,.01],[0,1,.01],
                               [0,0,.03],[1,0,.03],[1,1,.03],[0,1,.03]])
        # Both top sheets share a bridge far from the queried interior location.
        f=np.array([[0,1,2],[0,2,3],[4,6,5],[4,7,6],[1,5,6],[1,6,2]])
        target=Mesh(v,f,closed=False)
        a=SurfaceAnchor.on(src,0,[.5,.1,.4])
        with self.assertRaisesRegex(CorrespondenceError,'Ambiguous'):
            project_reference(a,src,target,.1,max_normal_angle=180)
