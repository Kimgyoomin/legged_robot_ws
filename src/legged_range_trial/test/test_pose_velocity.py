import math
import unittest
from legged_range_trial.pose_velocity import PoseVelocity


class VelocityTest(unittest.TestCase):
    def test_forward_in_rotated_frame(self):
        fit = PoseVelocity()
        for i in range(6):
            t=i*.1
            v=fit.add(t,10.,-4.+.6*t,math.pi/2)
        for actual,expected in zip(v,(.6,0.,0.)):
            self.assertAlmostEqual(actual,expected)

    def test_wrapped_yaw(self):
        fit = PoseVelocity()
        for i in range(8):
            t=i*.1
            v=fit.add(t,0.,0.,math.remainder(3.+.8*t,2*math.pi))
        self.assertAlmostEqual(v[2],.8)

    def test_time_jump_requires_new_samples(self):
        fit=PoseVelocity()
        for t in [10.,10.1,10.2,10.3]:fit.add(t,t,0.,0.)
        self.assertIsNone(fit.add(1.,0.,0.,0.))
        self.assertIsNone(fit.add(1.1,.1,0.,0.))
        self.assertAlmostEqual(fit.add(1.2,.2,0.,0.)[0],1.)
        self.assertIsNone(fit.add(2.,1.,0.,0.))

    def test_invalid_pose_is_not_a_zero_velocity(self):
        fit=PoseVelocity()
        for t in [0.,.1,.2,.3]:fit.add(t,0.,0.,0.)
        self.assertIsNone(fit.add(.4,10.,0.,0.))
        self.assertEqual(fit.reason,'pose_jump')
        self.assertIsNone(fit.add(.5,float('nan'),0.,0.))
        self.assertEqual(len(fit.samples),0)

    def test_duplicate_timestamp_resets_and_memory_is_bounded(self):
        fit=PoseVelocity()
        for i in range(10000):fit.add(i*.01,0.,0.,0.)
        self.assertLessEqual(len(fit.samples),42)
        self.assertIsNone(fit.add(99.99,0.,0.,0.))


if __name__ == '__main__':
    unittest.main()
