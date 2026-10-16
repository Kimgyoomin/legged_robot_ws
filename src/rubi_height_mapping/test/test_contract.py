import math
import struct
import unittest
from types import SimpleNamespace as NS
import numpy as np
from rubi_height_mapping.pointcloud import pointcloud_xyz, quaternion_matrix
from rubi_height_mapping.parameter import Parameter
from rubi_height_mapping.stamped_tf_queue import PendingPointCloud, PendingPointCloudQueue

class Contract(unittest.TestCase):
    def cloud(self):
        data=bytearray(80)
        for offset,xyz in zip([0,16,40,56],[(1,2,3),(4,5,6),(7,8,9),(float('nan'),0,0)]):
            struct.pack_into('<fff',data,offset,*xyz)
        return NS(is_bigendian=False,width=2,height=2,point_step=16,row_step=40,data=data,
                  fields=[NS(name=n,datatype=7,count=1,offset=4*i) for i,n in enumerate('xyz')])
    def test_organized_padding_and_nan(self):
        np.testing.assert_array_equal(pointcloud_xyz(self.cloud()),[[1,2,3],[4,5,6],[7,8,9]])
    def test_bad_layout_rejected(self):
        for mutate in [lambda c:setattr(c,'row_step',31),lambda c:c.data.pop(),
                       lambda c:setattr(c,'is_bigendian',True),lambda c:setattr(c.fields[2],'offset',15),
                       lambda c:c.fields.append(c.fields[0])]:
            c=self.cloud();mutate(c)
            with self.assertRaises(ValueError):pointcloud_xyz(c)
    def test_quaternion_axes_and_normalization(self):
        r=quaternion_matrix([0,0,2*math.sin(math.pi/4),2*math.cos(math.pi/4)])
        np.testing.assert_allclose(r[:3,:3]@[1,0,0],[0,1,0],atol=1e-12)
        with self.assertRaises(ValueError):quaternion_matrix([0,0,0,0])
    def test_geometry_rejected_before_allocation(self):
        for kwargs in [dict(resolution=0),dict(map_length=float('nan')),dict(map_length=8.025),dict(map_length=1000)]:
            with self.assertRaises(ValueError):Parameter(**kwargs).update()
    def test_ray_parameters_are_bounded(self):
        for kwargs in (dict(ray_mode='invalid'), dict(ray_voxel_size=-.01), dict(ray_voxel_size=.1), dict(ray_voxel_size=.001)):
            with self.assertRaises(ValueError): Parameter(**kwargs).update()
        Parameter(ray_mode='voxel',ray_voxel_size=.025).update()
    def test_queue_bounds_and_order(self):
        q=PendingPointCloudQueue(2,20)
        for t in (1,2,3):r,d=q.push(PendingPointCloud(None,'lidar',t,0,8))
        self.assertEqual(len(d),1);self.assertEqual(q.peek().stamp_ns,2)
        self.assertEqual(q.push(PendingPointCloud(None,'lidar',3,0,8))[0],'duplicate')
        self.assertEqual(q.push(PendingPointCloud(None,'lidar',1,0,8))[0],'out_of_order')
        self.assertEqual(q.payload_bytes,16)
if __name__=='__main__':unittest.main()
