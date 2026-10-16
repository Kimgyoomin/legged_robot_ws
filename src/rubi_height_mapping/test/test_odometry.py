import unittest
import numpy as np
from rubi_height_mapping.odometry import OdometryBuffer, compose_pose, slerp
from rubi_height_mapping.pointcloud import quaternion_matrix


class OdometryContract(unittest.TestCase):
    def test_interpolation_large_integer_timestamps_and_antipodal_quaternions(self):
        b = OdometryBuffer('world')
        start = 1780000000123456789
        b.add(start, 'world', 'BODY', [1, 2, 3], [0, 0, 0, 2])
        with self.assertRaises(LookupError): b.lookup(start+50000000)
        b.add(start+100000000, 'world', 'BODY', [3, 4, 5], [0, 0, -1, -1])
        p = b.lookup(start+50000000)
        np.testing.assert_allclose(p.position, [2, 3, 4])
        np.testing.assert_allclose(quaternion_matrix(p.quaternion)[:3, :3]@[1, 0, 0],
                                   [2**-.5, 2**-.5, 0], atol=1e-12)
        np.testing.assert_allclose(slerp([0,0,0,1], [0,0,0,-1], .5), [0,0,0,1])
        with self.assertRaises(LookupError): b.lookup(start-1)
        with self.assertRaises(LookupError): b.lookup(start+100000001)

    def test_delayed_order_gaps_and_duplicate(self):
        b = OdometryBuffer('map')
        for t in (1000000000, 1400000000): b.add(t, 'map', 'body', [0,0,0], [0,0,0,1])
        with self.assertRaises(LookupError): b.lookup(1100000000)
        b.add(1200000000, 'map', 'body', [2,0,0], [0,0,0,1])
        np.testing.assert_equal(b.lookup(1100000000).position, [1,0,0])
        self.assertFalse(b.add(1200000000, 'map', 'body', [100,0,0], [0,0,0,1]))
        np.testing.assert_equal(b.lookup(1200000000).position, [2,0,0])

    def test_frames_and_invalid_values_do_not_poison_buffer(self):
        b = OdometryBuffer('map')
        for parent, child, p, q in [('world','BODY',[0,0,0],[0,0,0,1]),
                ('map','',[0,0,0],[0,0,0,1]), ('map','map',[0,0,0],[0,0,0,1]),
                ('map','body',[np.nan,0,0],[0,0,0,1]), ('map','body',[0,0,0],[0,0,0,0])]:
            with self.assertRaises(ValueError): b.add(100,parent,child,p,q)
        self.assertIsNone(b.child_frame)
        b.add(100, 'map', 'body', [0,0,0], [0,0,0,1])
        with self.assertRaises(ValueError): b.add(101,'map','other',[0,0,0],[0,0,0,1])
        self.assertEqual(b.stamps,[100])
        b.clear()
        with self.assertRaises(LookupError): b.lookup(100)

    def test_cache_bounds(self):
        b = OdometryBuffer('map', cache_seconds=1, max_samples=3)
        for t in range(10): b.add(t*100000000,'map','body',[0,0,0],[0,0,0,1])
        self.assertEqual(b.stamps,[700000000,800000000,900000000])
        b.add(3000000000,'map','body',[0,0,0],[0,0,0,1])
        self.assertEqual(b.stamps,[3000000000])
        self.assertFalse(b.add(1,'map','body',[0,0,0],[0,0,0,1]))

    def test_upside_down_sensor_and_rotated_mount_translation(self):
        p,q=compose_pose([1,2,3],[0,0,1,1],[.2,0,.1],[1,0,0,0])
        np.testing.assert_allclose(p,[1,2.2,3.1],atol=1e-12)
        np.testing.assert_allclose(quaternion_matrix(q)[:3,:3]@[1,2,3],[2,1,-3],atol=1e-12)

    def test_invalid_configuration(self):
        for kwargs in [dict(max_gap_seconds=0),dict(cache_seconds=float('nan')),
                       dict(cache_seconds=.1,max_gap_seconds=.2),dict(max_samples=1)]:
            with self.assertRaises(ValueError): OdometryBuffer('map',**kwargs)

if __name__ == '__main__': unittest.main()
