#!/usr/bin/env python3
"""Offline RRT* on a measured OCGM; export ordered [x, y, theta] arrays.

The search space is XY for a circular robot that can turn in place. Start and
goal headings are retained for execution; length is the cost being optimized.
Only the measured probability CSV is used, never Gazebo/SDF obstacle geometry.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import secrets
import time

import numpy as np
from scipy.ndimage import distance_transform_edt, label

ROOT = Path(__file__).resolve().parent


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class GridMap:
    """Conservative disk configuration space with unknown cells blocked."""
    def __init__(self, probability, metadata, robot_radius=.5, margin=.10, free_threshold=.35):
        self.probability = np.asarray(probability, dtype=float)
        if self.probability.ndim != 2 or min(self.probability.shape) < 2:
            raise ValueError('OCGM must be a 2D probability grid')
        if not np.isfinite(self.probability).all() or not ((self.probability >= 0) & (self.probability <= 1)).all():
            raise ValueError('OCGM probabilities must be finite and within [0, 1]')
        if not all(math.isfinite(v) and v >= 0 for v in (robot_radius, margin)):
            raise ValueError('Robot radius and margin must be finite and nonnegative')
        if not math.isfinite(free_threshold) or not 0 < free_threshold < .5:
            raise ValueError('Free threshold must be between 0 and 0.5; unknown stays blocked')
        self.resolution = float(metadata['resolution'])
        self.extent = np.asarray(metadata['extent'], dtype=float)
        if (self.extent.shape != (4,) or not np.isfinite(self.extent).all() or
                not math.isfinite(self.resolution) or self.resolution <= 0 or
                self.extent[1] <= self.extent[0] or self.extent[3] <= self.extent[2]):
            raise ValueError('Invalid map extent/resolution')
        if list(self.probability.shape) != metadata['shape']:
            raise ValueError('CSV shape differs from map metadata')
        ny, nx = self.probability.shape
        if not np.allclose([(self.extent[1]-self.extent[0])/nx,
                            (self.extent[3]-self.extent[2])/ny], self.resolution, rtol=0, atol=1e-8):
            raise ValueError('Extent, resolution and shape are inconsistent')
        if not str(metadata.get('layout', '')).startswith('rows increase world Y; columns increase world X'):
            raise ValueError('Expected OCGM rows increasing world Y; do not guess/flip array orientation')
        self.robot_radius, self.margin, self.free_threshold = robot_radius, margin, free_threshold
        known_free = self.probability < free_threshold
        # Pad the outside as blocked. EDT measures distances between cell centres;
        # subtract two half diagonals to cover both the blocked square and any
        # point in a traversed free square, rather than just their cell centres.
        distances = distance_transform_edt(np.pad(known_free, 1, constant_values=False))[1:-1, 1:-1]
        self.clearance = np.maximum(0., distances * self.resolution - self.resolution * math.sqrt(2))
        self.free = known_free & (self.clearance > robot_radius + margin)
        self.labels, self.component_count = label(self.free)  # Four-connected: no diagonal squeezing.
        self.cells = np.argwhere(self.free)
        if not len(self.cells):
            raise ValueError('No known free space remains after inflating obstacles for the robot')
        self.centres = np.column_stack((self.extent[0] + (self.cells[:, 1] + .5)*self.resolution,
                                        self.extent[2] + (self.cells[:, 0] + .5)*self.resolution))

    @classmethod
    def load(cls, path, **settings):
        path = Path(path)
        metadata = json.loads(path.with_suffix('.meta.json').read_text())
        return cls(np.loadtxt(path, delimiter=','), metadata, **settings)

    def cell(self, xy):
        x, y = xy
        return (math.floor((x-self.extent[0])/self.resolution),
                math.floor((y-self.extent[2])/self.resolution))

    def is_free(self, xy):
        if len(xy) != 2 or not np.isfinite(xy).all():
            return False
        x, y = self.cell(xy)
        return 0 <= y < self.free.shape[0] and 0 <= x < self.free.shape[1] and bool(self.free[y, x])

    def component(self, xy):
        if not self.is_free(xy):
            raise ValueError(f'Pose {list(xy)} is occupied, unknown, outside, or too close to an obstacle')
        x, y = self.cell(xy)
        return int(self.labels[y, x])

    def segment_free(self, a, b):
        """Check every cell touched, including both sides at a grid corner/edge.

        Probe all grid-line crossings plus interval midpoints. Tiny offsets include
        neighbouring cells at an exact boundary, avoiding sparse-sampling shortcuts.
        """
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        if a.shape != (2,) or b.shape != (2,) or not self.is_free(a) or not self.is_free(b):
            return False
        start = (a - self.extent[[0, 2]]) / self.resolution
        end = (b - self.extent[[0, 2]]) / self.resolution
        delta = end - start
        times = [np.array([0., 1.])]
        for axis in (0, 1):
            if abs(delta[axis]) > 1e-12:
                lines = np.arange(math.floor(min(start[axis], end[axis]))+1,
                                  math.ceil(max(start[axis], end[axis])))
                times.append((lines-start[axis])/delta[axis])
        crossings = np.unique(np.concatenate(times))
        times = np.concatenate((crossings, (crossings[:-1]+crossings[1:])/2))
        points = start + times[:, None]*delta
        for offset in ((-1e-9,-1e-9), (-1e-9,1e-9), (1e-9,-1e-9), (1e-9,1e-9)):
            cells = np.floor(points + offset).astype(int)
            x, y = cells[:, 0], cells[:, 1]
            if (np.any(x < 0) or np.any(y < 0) or np.any(x >= self.free.shape[1]) or
                    np.any(y >= self.free.shape[0]) or not self.free[y, x].all()):
                return False
        return True

    def random_pair(self, rng, minimum_distance=5., start=None, goal=None):
        """Uniform free-cell samples in a common component; random heading in [-pi, pi]."""
        if not math.isfinite(minimum_distance) or minimum_distance < 0:
            raise ValueError('Minimum start-goal distance must be nonnegative')
        for pose in (start, goal):
            if pose is not None:
                if len(pose) != 3 or not np.isfinite(pose).all():
                    raise ValueError('Each pose must be finite [x, y, theta]')
                self.component(pose[:2])
        if start is not None and goal is not None:
            if self.component(start[:2]) != self.component(goal[:2]):
                raise ValueError('Start and goal lie in disconnected free-space components')
            if math.dist(start[:2], goal[:2]) < 1e-6:
                raise ValueError('Start and goal must have distinct XY positions')
            return np.array([*start[:2], wrap(start[2])]), np.array([*goal[:2], wrap(goal[2])])
        component_ids = self.labels[self.cells[:, 0], self.cells[:, 1]]
        fixed = start if start is not None else goal
        if fixed is not None:
            choices = np.flatnonzero(component_ids == self.component(fixed[:2]))
        else:
            choices = np.arange(len(self.centres))
        for _ in range(1000):
            first = np.asarray(fixed[:2]) if fixed is not None else self.centres[rng.choice(choices)]
            available = choices[(component_ids[choices] == self.component(first)) &
                                (np.linalg.norm(self.centres[choices]-first, axis=1) >= max(minimum_distance, 1e-6))]
            if len(available):
                second = self.centres[rng.choice(available)]
                random_first = np.array([*first, rng.uniform(-math.pi, math.pi)])
                random_second = np.array([*second, rng.uniform(-math.pi, math.pi)])
                if start is not None:
                    return np.array([*start[:2], wrap(start[2])]), random_second
                if goal is not None:
                    return random_second, np.array([*goal[:2], wrap(goal[2])])
                return random_first, random_second
        raise ValueError('Cannot sample connected poses at this separation; reduce --min-distance or collect more map')


class RRTStar:
    def __init__(self, grid, rng, iterations=5000, step_size=.65, neighbour_radius=2., goal_bias=.12):
        if not isinstance(iterations, int) or iterations < 1:
            raise ValueError('Iterations must be a positive integer')
        if not all(math.isfinite(v) and v > 0 for v in (step_size, neighbour_radius)):
            raise ValueError('Step size and neighbour radius must be positive')
        if not math.isfinite(goal_bias) or not 0 <= goal_bias < 1:
            raise ValueError('Goal bias must be within [0, 1)')
        self.grid, self.rng, self.iterations = grid, rng, iterations
        self.step_size, self.neighbour_radius, self.goal_bias = step_size, neighbour_radius, goal_bias
        self.points = np.empty((iterations+1, 2))
        self.parents = np.full(iterations+1, -1, dtype=int)
        self.costs = np.zeros(iterations+1)
        self.children = [set() for _ in range(iterations+1)]
        self.count, self.rewires, self.history = 1, 0, []
        self.goal_candidates = []

    def reparent(self, node, parent):
        old = self.parents[node]
        delta = self.costs[parent] + np.linalg.norm(self.points[node]-self.points[parent]) - self.costs[node]
        self.children[old].remove(node)
        self.parents[node] = parent
        self.children[parent].add(node)
        pending = [node]
        while pending:
            current = pending.pop()
            self.costs[current] += delta
            pending.extend(self.children[current])
        self.rewires += 1

    def plan(self, start, goal):
        self.points[0] = start[:2]
        self.grid.component(start[:2])
        if self.grid.component(start[:2]) != self.grid.component(goal[:2]):
            raise ValueError('Start and goal are not connected')
        component = self.grid.component(start[:2])
        cells = self.grid.cells
        samples = self.grid.centres[self.grid.labels[cells[:,0], cells[:,1]] == component]
        # Standard 2D shrinking RRT* neighbourhood, capped at the requested radius.
        gamma = 2.5 * math.sqrt(1.5 * len(samples) * self.grid.resolution**2 / math.pi)
        goal_xy = np.asarray(goal[:2])
        first_cost, first_iteration = None, None
        if math.dist(start[:2], goal[:2]) <= self.step_size and self.grid.segment_free(start[:2], goal[:2]):
            self.goal_candidates.append(0)
        for iteration in range(1, self.iterations+1):
            target = goal_xy if self.rng.random() < self.goal_bias else samples[self.rng.integers(len(samples))]
            distances = np.linalg.norm(self.points[:self.count]-target, axis=1)
            nearest = int(np.argmin(distances))
            distance = distances[nearest]
            if distance < 1e-9:
                continue
            new = self.points[nearest] + (target-self.points[nearest])*min(1., self.step_size/distance)
            if not self.grid.segment_free(self.points[nearest], new):
                continue
            distances = np.linalg.norm(self.points[:self.count]-new, axis=1)
            if distances.min() < 1e-8:
                continue
            n = self.count+1
            radius = min(self.neighbour_radius, gamma*math.sqrt(math.log(n)/n))
            nearby = np.flatnonzero(distances <= radius)
            parent = nearest
            cost = self.costs[nearest]+distances[nearest]
            alternatives = self.costs[nearby]+distances[nearby]
            for candidate in nearby[np.argsort(alternatives)]:
                possible = self.costs[candidate]+distances[candidate]
                if possible >= cost-1e-10:
                    break
                if self.grid.segment_free(self.points[candidate], new):
                    parent, cost = int(candidate), float(possible)
                    break
            node = self.count
            self.count += 1
            self.points[node], self.parents[node], self.costs[node] = new, parent, cost
            self.children[parent].add(node)
            ancestors = {node}
            ancestor = parent
            while ancestor >= 0:
                ancestors.add(ancestor)
                ancestor = self.parents[ancestor]
            for candidate in nearby:
                if candidate in ancestors:
                    continue
                possible = cost+distances[candidate]
                if possible < self.costs[candidate]-1e-10 and self.grid.segment_free(new, self.points[candidate]):
                    self.reparent(int(candidate), node)
            if np.linalg.norm(new-goal_xy) <= self.step_size and self.grid.segment_free(new, goal_xy):
                self.goal_candidates.append(node)
            if self.goal_candidates:
                best = min(self.costs[i]+np.linalg.norm(self.points[i]-goal_xy) for i in self.goal_candidates)
                if first_cost is None:
                    first_cost, first_iteration = float(best), iteration
                if not self.history or best < self.history[-1][1]-1e-9:
                    self.history.append((iteration, float(best)))
        if not self.goal_candidates:
            raise RuntimeError(f'No path after {self.iterations} iterations; try more iterations or another seed')
        best_node = min(self.goal_candidates, key=lambda i: self.costs[i]+np.linalg.norm(self.points[i]-goal_xy))
        indices = [best_node]
        while indices[-1] != 0:
            indices.append(int(self.parents[indices[-1]]))
        indices.reverse()
        path = self.points[indices].copy()
        if np.linalg.norm(path[-1]-goal_xy) > 1e-8:
            path = np.vstack((path, goal_xy))
        path[0], path[-1] = start[:2], goal[:2]
        return path, indices, dict(iterations=self.iterations, tree_nodes=self.count, rewires=self.rewires,
                                  first_solution_iteration=first_iteration, first_solution_length_m=first_cost,
                                  path_length_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()))


def pose_path(xy, start_heading, goal_heading):
    poses = np.empty((len(xy),3))
    poses[:,:2] = xy
    differences = np.diff(xy,axis=0)
    poses[:-1,2] = np.arctan2(differences[:,1],differences[:,0])
    poses[0,2], poses[-1,2] = start_heading, goal_heading
    return poses


def interpolate(poses, spacing=.10):
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('Interpolation spacing must be positive')
    rows = [poses[0].copy()]
    for a,b in zip(poses[:-1],poses[1:]):
        steps = max(1,math.ceil(math.dist(a[:2],b[:2])/spacing))
        for i in range(1,steps+1):
            u = i/steps
            rows.append([a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]), wrap(a[2]+u*wrap(b[2]-a[2]))])
    result = np.asarray(rows)
    result[0],result[-1] = poses[0],poses[-1]
    return result


def plot_result(grid, tree, poses, output, seed, metrics, show=False):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, axes = plt.subplots(1,2,figsize=(15,7.8))
    fig.subplots_adjust(left=.055,right=.985,bottom=.14,top=.84,wspace=.13)
    fig.patch.set_facecolor('#f5f7fa')
    fig.suptitle(f'RRT*  |  Measured occupancy grid  |  seed {seed}',fontsize=17,fontweight='bold',y=.96)
    edges = [[tree.points[i],tree.points[tree.parents[i]]] for i in range(1,tree.count)]
    for ax in axes:
        ax.imshow(grid.probability,origin='lower',extent=grid.extent,cmap='gray_r',vmin=0,vmax=1,interpolation='nearest')
        inflation = (grid.probability < grid.free_threshold) & ~grid.free
        overlay = np.zeros((*inflation.shape,4))
        overlay[inflation] = (.95,.49,.32,.28)
        ax.imshow(overlay,origin='lower',extent=grid.extent,interpolation='nearest')
        ax.plot(poses[:,0],poses[:,1],color='#007f88',lw=2.4,zorder=4,label='Ordered RRT* path')
        for pose,marker,color,name in [(poses[0],'o','#22a86b','Start'),(poses[-1],'*','#dd6157','Goal')]:
            ax.scatter(pose[0],pose[1],marker=marker,c=color,s=110,edgecolors='white',linewidths=1,zorder=6,label=name)
            ax.arrow(pose[0],pose[1],.6*math.cos(pose[2]),.6*math.sin(pose[2]),width=.025,
                     head_width=.18,color=color,zorder=6,length_includes_head=True)
        ax.set(xlabel='World X (m)',ylabel='World Y (m)',aspect='equal')
        ax.spines[['top','right']].set_visible(False)
    axes[0].add_collection(LineCollection(edges,colors='#2b967c',linewidths=.35,alpha=.24,zorder=2))
    axes[0].set_title(f'Search tree  /  {tree.count:,} nodes  /  {tree.rewires:,} rewires',pad=12)
    axes[1].scatter(poses[:,0],poses[:,1],s=16,c='#007f88',edgecolors='white',linewidths=.3,zorder=5)
    axes[1].set_title(f'Execution order  /  {len(poses)} nodes  /  {metrics["path_length_m"]:.2f} m',pad=12)
    axes[1].legend(loc='upper right',framealpha=.95,fontsize=9)
    fig.text(.5,.035,f'Orange tint: robot clearance buffer | Unknown (grey): blocked | '
             f'Robot radius {grid.robot_radius:.2f} m + margin {grid.margin:.2f} m',fontsize=10,ha='center')
    fig.savefig(output,dpi=170,facecolor=fig.get_facecolor())
    if show:
        plt.show()
    plt.close(fig)


def load_plan(path):
    """Revalidate the map identity and every exported segment before execution."""
    path = Path(path)
    plan = json.loads(path.read_text())
    if plan.get('algorithm') != 'RRT*' or plan.get('frame') != 'world':
        raise ValueError('Expected a world-frame RRT* plan')
    map_path = Path(plan['map']['path'])
    if digest(map_path) != plan['map']['sha256'] or digest(map_path.with_suffix('.meta.json')) != plan['map']['metadata_sha256']:
        raise ValueError('Map changed after planning; generate a fresh RRT* plan')
    grid = GridMap.load(map_path,**plan['configuration_space'])
    poses = np.asarray(plan['path_nodes'],dtype=float)
    if poses.ndim != 2 or poses.shape[1] != 3 or len(poses)<2 or not np.isfinite(poses).all():
        raise ValueError('Invalid ordered pose array')
    if not np.allclose(poses[0],plan['start'],atol=1e-10,rtol=0) or not np.allclose(poses[-1],plan['goal'],atol=1e-10,rtol=0):
        raise ValueError('Path endpoints differ from the requested poses')
    if any(not grid.segment_free(a[:2],b[:2]) for a,b in zip(poses[:-1],poses[1:])):
        raise ValueError('Path intersects occupied, unknown, or inflated cells')
    return plan,grid,poses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,default=ROOT/'ocgm_660610823.csv')
    parser.add_argument('--seed',type=int,help='Repeat the same random experiment')
    parser.add_argument('--iterations',type=int,default=5000)
    parser.add_argument('--robot-radius',type=float,default=.50,help='Conservative circular footprint radius, metres')
    parser.add_argument('--margin',type=float,default=.10)
    parser.add_argument('--free-threshold',type=float,default=.35)
    parser.add_argument('--min-distance',type=float,default=5.)
    parser.add_argument('--step-size',type=float,default=.65)
    parser.add_argument('--neighbour-radius',type=float,default=2.)
    parser.add_argument('--goal-bias',type=float,default=.12)
    parser.add_argument('--spacing',type=float,default=.10)
    parser.add_argument('--start',type=float,nargs=3,metavar=('X','Y','THETA'))
    parser.add_argument('--goal',type=float,nargs=3,metavar=('X','Y','THETA'))
    parser.add_argument('--output-dir',type=Path,help='New directory; existing results are never overwritten')
    parser.add_argument('--show',action='store_true',help='Show the result plot after planning')
    args = parser.parse_args()
    seed = args.seed if args.seed is not None else secrets.randbits(32)
    if seed < 0:
        parser.error('Seed must be nonnegative')
    try:
        rng = np.random.default_rng(seed)
        map_path = args.map.resolve()
        settings = dict(robot_radius=args.robot_radius,margin=args.margin,free_threshold=args.free_threshold)
        grid = GridMap.load(map_path,**settings)
        start,goal = grid.random_pair(rng,args.min_distance,args.start,args.goal)
        tree = RRTStar(grid,rng,args.iterations,args.step_size,args.neighbour_radius,args.goal_bias)
        print(f'Map: {map_path}\nSeed: {seed}\nStart [x,y,theta]: {start.tolist()}\nGoal  [x,y,theta]: {goal.tolist()}',flush=True)
        now = time.monotonic()
        xy,indices,metrics = tree.plan(start,goal)
        metrics['planning_seconds'] = time.monotonic()-now
        poses = pose_path(xy,start[2],goal[2])
        dense = interpolate(poses,args.spacing)
        assert all(grid.segment_free(a[:2],b[:2]) for a,b in zip(xy[:-1],xy[1:]))
        output = args.output_dir or ROOT/'planning'/f'rrtstar_{datetime.now():%Y%m%d_%H%M%S_%f}_seed{seed}'
        output.mkdir(parents=True,exist_ok=False)
        plan = dict(algorithm='RRT*',frame='world',units=dict(x='metres',y='metres',theta='radians'),seed=seed,
                    map=dict(path=str(map_path),sha256=digest(map_path),metadata_sha256=digest(map_path.with_suffix('.meta.json'))),
                    configuration_space=settings,unknown_policy='blocked',state_space='XY with in-place rotations; circular footprint',
                    start=start.tolist(),goal=goal.tolist(),path_nodes=poses.tolist(),tree_path_indices=indices,
                    interpolation_spacing_m=args.spacing,planner=dict(iterations=args.iterations,step_size=args.step_size,
                    neighbour_radius=args.neighbour_radius,goal_bias=args.goal_bias),metrics=metrics)
        (output/'plan.json').write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n')
        for name,data in [('path_nodes',poses),('path_interpolated',dense)]:
            np.save(output/(name+'.npy'),data)
            np.savetxt(output/(name+'.csv'),data,delimiter=',',header='x,y,theta',comments='',fmt='%.10f')
        np.savetxt(output/'tree.csv',np.column_stack((np.arange(tree.count),tree.points[:tree.count],tree.parents[:tree.count],tree.costs[:tree.count])),
                   delimiter=',',header='node,x,y,parent,cost_m',comments='',fmt=['%d','%.10f','%.10f','%d','%.10f'])
        np.savetxt(output/'cost_history.csv',np.asarray(tree.history),delimiter=',',header='iteration,best_path_length_m',comments='',fmt='%.10f')
        plot_result(grid,tree,poses,output/'rrt_star.png',seed,metrics,args.show)
        load_plan(output/'plan.json')
        print(f'RRT*: {metrics["path_length_m"]:.3f} m, {len(poses)} ordered nodes, {tree.rewires} rewires, '
              f'{metrics["planning_seconds"]:.2f} seconds\nSaved: {output.resolve()}',flush=True)
    except (ValueError,RuntimeError,OSError,KeyError) as error:
        parser.exit(1,f'RRT* failed: {error}\n')


if __name__ == '__main__':
    main()
