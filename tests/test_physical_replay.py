"""Recorded state replay must preserve units, rotation, independence and scope."""
from copy import deepcopy
import unittest

import numpy as np

from mechanism_lab.physical_replay import build_assembly, Y_UP_TO_Z_UP, pose


def fixtures():
    meshes = {'units':'mm','meshes':[
        dict(name='part',body='arm',vertices_mm=[[1,0,0],[0,1,0],[0,0,1]],triangles=[[0,1,2]]),
        dict(name='free',body='payload',vertices_mm=[[1,0,0],[0,1,0],[0,0,1]],triangles=[[0,1,2]])]}
    rotation = [0,-1,0,1,0,0,0,0,1]
    frames = []
    for time in (0.,.5,1.):
        frames.append(dict(time_s=time,bodies={
            'moving':dict(position_m=[time,.2,.3],rotation_row_major=rotation),
            'freebody':dict(position_m=[.7,.4-time*.1,0],rotation_row_major=np.eye(3).ravel().tolist())}))
    result = dict(schema='cybrgeo-causal-simulation/1',frames=frames,model_sha256='model',control_sha256='control')
    return meshes, {'arm':'moving','payload':'freebody'}, result


class PhysicalReplayTests(unittest.TestCase):
    def build(self, mutation=None):
        args = fixtures()
        if mutation:
            mutation(*args)
        return build_assembly(*args,world_rotation=Y_UP_TO_Z_UP)

    def test_recorded_body_vertex_matches_si_world_conversion(self):
        assembly = self.build()
        part = assembly.parts[0]
        expected_rotation = Y_UP_TO_Z_UP @ np.array([[0,-1,0],[1,0,0],[0,0,1]])
        for time in (0.,.5,1.):
            matrix = assembly.pose(part,time)
            np.testing.assert_allclose(matrix[:3,:3],expected_rotation,atol=1e-12)
            np.testing.assert_allclose(matrix[:3,3],[time*1000,-300,200],atol=1e-12)
            vertex = np.array([1.,0.,0.])
            np.testing.assert_allclose(matrix[:3,:3]@vertex+matrix[:3,3],
                Y_UP_TO_Z_UP@(np.array([0.,1.,0.])+np.array([time,.2,.3])*1000))

    def test_free_body_is_not_attached_to_arm(self):
        assembly = self.build()
        arm,payload = assembly.parts
        np.testing.assert_allclose(assembly.pose(payload,.5)[:3,3],[700,0,350])
        self.assertFalse(np.allclose(assembly.pose(payload,.5),assembly.pose(arm,.5)))

    def test_between_samples_holds_preceding_recorded_pose(self):
        assembly = self.build()
        part = assembly.parts[0]
        np.testing.assert_array_equal(assembly.pose(part,.49999),assembly.pose(part,0))
        np.testing.assert_array_equal(assembly.pose(part,.50001),assembly.pose(part,.5))
        self.assertEqual(assembly.metadata['recorded_times_s'],[0,.5,1])

    def test_pose_copy_cannot_mutate_recorded_state(self):
        assembly = self.build()
        assembly.pose(assembly.parts[0],0)[0,3] = 999
        self.assertEqual(assembly.pose(assembly.parts[0],0)[0,3],0)

    def test_exploded_and_nonfinite_replay_rejected(self):
        assembly = self.build()
        for time,explode in [(0,1),(float('nan'),0)]:
            with self.assertRaises(ValueError):
                assembly.pose(assembly.parts[0],time,explode)

    def test_missing_body_rejected(self):
        with self.assertRaisesRegex(ValueError,'Missing recorded body'):
            self.build(lambda m,b,s:s['frames'][1]['bodies'].pop('freebody'))

    def test_nonrotation_rejected(self):
        with self.assertRaisesRegex(ValueError,'proper rotation'):
            self.build(lambda m,b,s:s['frames'][0]['bodies']['moving'].update(rotation_row_major=[0]*9))

    def test_reflection_basis_rejected(self):
        with self.assertRaisesRegex(ValueError,'proper rotation'):
            build_assembly(*fixtures(),world_rotation=np.diag([-1,1,1]))

    def test_unmapped_geometry_rejected(self):
        with self.assertRaisesRegex(ValueError,'Unmapped'):
            self.build(lambda m,b,s:b.pop('payload'))

    def test_units_and_summary_rejected(self):
        for mutate in [lambda m,b,s:m.update(units='m'),lambda m,b,s:s.update(frames=1201)]:
            with self.assertRaises(ValueError):
                self.build(mutate)

    def test_duplicate_time_rejected(self):
        with self.assertRaisesRegex(ValueError,'strictly increasing'):
            self.build(lambda m,b,s:s['frames'][1].update(time_s=0))

    def test_invalid_triangle_rejected(self):
        with self.assertRaisesRegex(ValueError,'Invalid triangle'):
            self.build(lambda m,b,s:m['meshes'][0].update(triangles=[[0,1,9]]))

    def test_serialized_cache_restores_motion_and_rejects_changed_state(self):
        import json
        assembly = self.build()
        expected = assembly.pose(assembly.parts[0],.5)
        assembly.metadata = json.loads(json.dumps(assembly.metadata))
        assembly.motion_function = pose.bind(assembly)
        np.testing.assert_array_equal(assembly.pose(assembly.parts[0],.5),expected)
        assembly.metadata['recorded_body_transforms_mm']['moving'][1][0][3] += 1
        with self.assertRaisesRegex(ValueError,'identity mismatch'):
            pose.bind(assembly)

    def test_cache_rejects_reattached_payload(self):
        assembly = self.build()
        assembly.parts[1].motion = 'moving'
        with self.assertRaisesRegex(ValueError,'bindings changed'):
            pose.bind(assembly)


if __name__ == '__main__':
    unittest.main()
