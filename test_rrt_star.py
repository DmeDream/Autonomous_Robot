"""Planning and controller checks; no simulator, hardware or network connection."""
import math
import unittest

import numpy as np

from follow_rrt_coppelia import from_scene, to_scene, validate_config
from rrt_path_control import PathFollower, wheel_velocities
from rrt_star_plan import GridMap, RRTStar, interpolate, pose_path, wrap


def metadata(array, resolution=1.):
    return dict(resolution=resolution,extent=[0,array.shape[1]*resolution,0,array.shape[0]*resolution],
                shape=list(array.shape),layout='rows increase world Y; columns increase world X; cell-center coordinates')


def intersects_box(a,b,xlo,xhi,ylo,yhi):
    """Independent closed-segment/AABB slab calculation, without raster traversal."""
    lo,hi = 0.,1.
    for origin,end,minimum,maximum in zip(a,b,(xlo,ylo),(xhi,yhi)):
        delta = end-origin
        if abs(delta)<1e-12:
            if origin<minimum-1e-10 or origin>maximum+1e-10:
                return False
        else:
            times = sorted(((minimum-origin)/delta,(maximum-origin)/delta))
            lo,hi = max(lo,times[0]),min(hi,times[1])
            if lo>hi+1e-10:
                return False
    return True


class GridChecks(unittest.TestCase):
    def test_unknown_inflation_and_finite_data(self):
        p = np.full((30,30),.02)
        p[15,15] = .5
        grid = GridMap(p,metadata(p,.2),robot_radius=.4,margin=.1)
        self.assertFalse(grid.is_free((3.1,3.1)))
        self.assertFalse(grid.is_free((3.5,3.1)))
        self.assertFalse(grid.is_free((.1,.1)))
        self.assertTrue(grid.is_free((1.5,1.5)))
        self.assertFalse(grid.segment_free((-1,1),(1,1)))
        self.assertFalse(grid.is_free((math.nan,1)))
        for value in (math.nan,math.inf,-.1,1.1):
            bad = p.copy()
            bad[0,0] = value
            with self.assertRaises(ValueError):
                GridMap(bad,metadata(bad,.2))
        wrong = metadata(p,.2)
        wrong['extent'][1] = 5
        with self.assertRaises(ValueError):
            GridMap(p,wrong)

    def test_supercover_against_geometric_oracle(self):
        # Isolate traversal from inflation, providing a configuration-space raster.
        grid = object.__new__(GridMap)
        grid.free = np.ones((8,8),dtype=bool)
        grid.free[3,3] = grid.free[5,4] = grid.free[1,6] = False
        grid.resolution,grid.extent = 1.,np.array([0,8,0,8])
        rng = np.random.default_rng(42)
        cases = [((1.5,1.5),(5.5,5.5)),((2,3),(3,2)),((3,2),(3,5)),
                 ((1,1),(2,1)),((3,3),(3,3)),((1.5,1.5),(1.5,1.5))]
        cases += [(rng.uniform(.1,7.9,2),rng.uniform(.1,7.9,2)) for _ in range(300)]
        blocked = [(3,3),(4,5),(6,1)]
        for a,b in cases:
            expected = not any(intersects_box(a,b,x,x+1,y,y+1) for x,y in blocked)
            self.assertEqual(grid.segment_free(a,b),expected,(a,b))

    def test_random_poses_reproducible_and_connected(self):
        p = np.full((40,40),.02)
        p[:,19:21] = .5
        grid = GridMap(p,metadata(p,.2),robot_radius=.1,margin=.1)
        for seed in range(15):
            start,goal = grid.random_pair(np.random.default_rng(seed),minimum_distance=2.)
            again = grid.random_pair(np.random.default_rng(seed),minimum_distance=2.)
            np.testing.assert_array_equal([start,goal],again)
            self.assertEqual(grid.component(start[:2]),grid.component(goal[:2]))
            self.assertGreaterEqual(math.dist(start[:2],goal[:2]),2.)
            self.assertTrue(-math.pi<=start[2]<=math.pi and -math.pi<=goal[2]<=math.pi)
        with self.assertRaises(ValueError):
            grid.random_pair(np.random.default_rng(1),start=[1,1,0],goal=[7,1,0])


class PlannerChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p = np.full((50,50),.02)
        p[:,25] = .99
        p[32:40,25] = .02
        cls.grid = GridMap(p,metadata(p,.2),robot_radius=.18,margin=.10)
        cls.start,cls.goal = np.array([1.5,3.1,.7]),np.array([8.5,3.1,-.6])
        cls.tree = RRTStar(cls.grid,np.random.default_rng(271411),iterations=2500)
        cls.xy,cls.indices,cls.metrics = cls.tree.plan(cls.start,cls.goal)

    def test_path_goes_through_gap_and_cost_improves(self):
        np.testing.assert_allclose(self.xy[0],self.start[:2])
        np.testing.assert_allclose(self.xy[-1],self.goal[:2])
        self.assertGreater(float(self.xy[:,1].max()),6.4)
        self.assertGreater(self.tree.rewires,0)
        self.assertLessEqual(self.metrics['path_length_m'],self.metrics['first_solution_length_m']+1e-8)
        self.assertTrue(all(self.grid.segment_free(a,b) for a,b in zip(self.xy[:-1],self.xy[1:])))
        # Independent distance-to-blocked-cell centres minus cell circumradius
        # is a conservative bound on distance to the actual obstacle squares.
        blocked = np.argwhere(self.grid.probability>=self.grid.free_threshold)
        centres = np.column_stack(((blocked[:,1]+.5)*.2,(blocked[:,0]+.5)*.2))
        for a,b in zip(self.xy[:-1],self.xy[1:]):
            delta = b-a
            u = np.clip((centres-a)@delta/(delta@delta),0,1)
            distances = np.linalg.norm(centres-(a+u[:,None]*delta),axis=1)-.2/math.sqrt(2)
            self.assertGreater(float(distances.min()),.18+.10-1e-8)

    def test_tree_costs_and_acyclic_parent_chain_after_rewiring(self):
        tree = self.tree
        for node in range(1,tree.count):
            current,total,visited = node,0.,set()
            while current!=0:
                self.assertNotIn(current,visited)
                visited.add(current)
                parent = tree.parents[current]
                self.assertTrue(0<=parent<tree.count)
                total += math.dist(tree.points[current],tree.points[parent])
                current = parent
            self.assertAlmostEqual(total,tree.costs[node],places=8)

    def test_controller_on_planned_path_ideal_differential_drive(self):
        poses = pose_path(self.xy,self.start[2],self.goal[2])
        follower = PathFollower(poses)
        pose = self.start.copy()
        dt = .05
        for _ in range(20000):
            linear,angular,done = follower.command(pose)
            if done:
                break
            # Integrate wheel-derived velocities using the exact constant-curvature arc.
            left,right = wheel_velocities(linear,angular,.1,.5)
            v,w = .1*(left+right)/2,.1*(right-left)/.5
            if abs(w)>1e-10:
                pose[0] += v/w*(math.sin(pose[2]+w*dt)-math.sin(pose[2]))
                pose[1] += v/w*(-math.cos(pose[2]+w*dt)+math.cos(pose[2]))
            else:
                pose[:2] += v*dt*np.array([math.cos(pose[2]),math.sin(pose[2])])
            pose[2] = wrap(pose[2]+w*dt)
            self.assertTrue(self.grid.is_free(pose[:2]))
        self.assertTrue(done,'Follower did not converge')
        self.assertLess(math.dist(pose[:2],self.goal[:2]),.04)
        self.assertLess(abs(wrap(pose[2]-self.goal[2])),.06)


class ControllerChecks(unittest.TestCase):
    def test_interpolation_endpoints_spacing_and_short_angle_wrap(self):
        poses = np.array([[0,0,math.pi-.02],[1,0,-math.pi+.02]])
        samples = interpolate(poses,.2)
        np.testing.assert_array_equal(samples[[0,-1]],poses)
        self.assertLessEqual(np.linalg.norm(np.diff(samples[:,:2],axis=0),axis=1).max(),.2+1e-10)
        self.assertTrue((np.abs(samples[:,2])>3).all())

    def test_wheel_speed_direction_units_and_saturation(self):
        np.testing.assert_allclose(wheel_velocities(.2,0,.1,.5),(2,2))
        np.testing.assert_allclose(wheel_velocities(0,1,.1,.5),(-2.5,2.5))
        left,right = wheel_velocities(.2,4,.1,.5,maximum=3)
        self.assertLessEqual(max(abs(left),abs(right)),3)
        self.assertAlmostEqual((left+right)/(right-left),.2/(4*.5/2))

    def test_frame_transform_roundtrip(self):
        config = dict(robot='/robot',left_joints=['/robot/left'],right_joints=['/robot/right'],
                      wheel_radius=.1,wheel_separation=.5,footprint_radius=.4,
                      map_to_scene=[2,-3,.4],robot_forward_yaw=math.pi/2)
        validate_config(config)
        for pose in ([0,0,0],[-3.2,5.7,-2.9]):
            scene = to_scene(pose,config)
            restored = from_scene([*scene[:2],0],[0,0,scene[2]],config)
            np.testing.assert_allclose(restored,pose,atol=1e-10)
        config['wheel_radius'] = None
        with self.assertRaises(ValueError):
            validate_config(config)


if __name__=='__main__':
    unittest.main()
