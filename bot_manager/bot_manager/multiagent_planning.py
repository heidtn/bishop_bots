from collections import namedtuple
from typing import List
import structures

import numpy as np
import fcl
import pyvista as pv
import tqdm
import heapq
from dataclasses import dataclass, field
import time

Bounds = namedtuple('Bounds', ["x_min", "y_min", "z_min", "x_max", "y_max", "z_max"])

class Roadmap:
    """
    Create a roadmap with collision checking against a list of static colliders
    """
    def __init__(self, bounds: Bounds, static_colliders: List[fcl.CollisionObject], clearance_size: float):
        self.vertices = []
        self.edges = []
        self.edges_lookup = {}
        self.bounds = bounds
        self.static_manager = fcl.DynamicAABBTreeCollisionManager()
        self.static_manager.registerObjects(static_colliders)
        
        self.vertex_manager = fcl.DynamicAABBTreeCollisionManager()
        self.edge_manager = fcl.DynamicAABBTreeCollisionManager()

        self.clearance = clearance_size

        self.static_manager.setup()
        self.vertex_manager.setup()
        self.edge_manager.setup()

        self.vertex_collision_map = {}
        self.stationary_vertices = []

    def add_travel_vertex(self, point: np.array, add_edge=True):
        sphere = fcl.Sphere(self.clearance)
        tf = fcl.Transform(point)
        col = fcl.CollisionObject(sphere, tf)

        if self.does_collide(col):
            return 0

        self.vertex_manager.registerObjects([col])
        self.vertices.append(point)
        self.vertex_collision_map[id(sphere)] = len(self.vertices) - 1
        new_idx = len(self.vertices) - 1

        if not add_edge:
            return new_idx

        self.add_edge(new_idx, point)
        return new_idx
    
    def add_stationary_vertex(self, point: np.array):
        idx = self.add_travel_vertex(point, add_edge=False)
        self.stationary_vertices.append(self.vertices[idx])
        return idx

    def add_edge(self, new_idx, point):
        # TODO clean up this magic bs ughhh 
        min_dist = 350.0 / np.log((len(self.vertices) + 1.0) / 3.0)
        for i, vert in enumerate(self.vertices[:-1]):
            distance = np.linalg.norm(vert - point)
            if distance < min_dist:
                node0 = vert
                node1 = point              
                cyl, col = structures.cyl_from_points(node0, node1, self.clearance)
                
                exclusions = [i, new_idx] 
                if self.collides_excluding_vertices(col, exclusions):
                    continue
                
                self.edges.append((new_idx, i))
                if new_idx not in self.edges_lookup:
                    self.edges_lookup[new_idx] = [i]
                else:
                    self.edges_lookup[new_idx].append(i)

                if i not in self.edges_lookup:
                    self.edges_lookup[i] = [new_idx]
                else:
                    self.edges_lookup[i].append(new_idx)

    def node_distance(self, node0, node1):
        return np.linalg.norm(self.vertices[node0] - self.vertices[node1])
    
    def does_collide(self, collision_object):
        req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
        rdata = fcl.CollisionData(request = req)
        self.static_manager.collide(collision_object, rdata, fcl.defaultCollisionCallback)
        if rdata.result.is_collision:
            return True
        
        self.vertex_manager.collide(collision_object, rdata, fcl.defaultCollisionCallback)
        if rdata.result.is_collision:
            return True

        self.edge_manager.collide(collision_object, rdata, fcl.defaultCollisionCallback)
        if rdata.result.is_collision:
            return True
        return False

    def collides_excluding_vertices(self, collision_object, exclusions):
        req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
        rdata = fcl.CollisionData(request = req)
        self.static_manager.collide(collision_object, rdata, fcl.defaultCollisionCallback)
        if rdata.result.is_collision:
            return True
        
        rdata = fcl.CollisionData(request = req)
        self.vertex_manager.collide(collision_object, rdata, fcl.defaultCollisionCallback)
        if rdata.result.is_collision:
            vert_cols = [self.vertex_collision_map[id(c.o1)] for c in rdata.result.contacts]
            #print("new check: ", vert_cols, exclusions)
            for contact in rdata.result.contacts:
                # Extract collision geometries that are in contact
                coll_geom_0 = contact.o1
                if self.vertex_collision_map[id(coll_geom_0)] not in exclusions:
                    return True
        
        rdata = fcl.CollisionData(request = req)
        self.edge_manager.collide(collision_object, rdata, fcl.defaultCollisionCallback)
        if rdata.result.is_collision:
            return True
        return False


    def add_random_vertex(self):
        connections = 0
        while connections == 0:
            x_rand = np.random.uniform(self.bounds.x_max, self.bounds.x_min)
            y_rand = np.random.uniform(self.bounds.y_max, self.bounds.y_min)
            z_rand = np.random.uniform(self.bounds.z_max, self.bounds.z_min)
            connections = self.add_travel_vertex(np.array([x_rand, y_rand, z_rand]))

    def visualize_roadmap(self, p: pv.Plotter, tubesize=None): 
        if not tubesize:
            tubesize = self.clearance
        raw_points = self.vertices
        sizes = np.ones(len(raw_points)) * self.clearance
        segments = []
        for pt in tqdm.tqdm(self.edges):
            #sph = pv.Sphere(nodesize, pt, theta_resolution=10, phi_resolution=10)
            segments.append(raw_points[pt[0]])
            segments.append(raw_points[pt[1]])
        seg_arr = np.array(segments)
        
        trusses = pv.line_segments_from_points(seg_arr)
        tube = trusses.tube(tubesize)
        p.add_mesh(tube, color="blue")

        pdata = pv.PolyData(raw_points, point_size=self.clearance)
        sphere = pv.Sphere(radius=self.clearance)
        pc = pdata.glyph(scale=False, geom=sphere)
        p.add_mesh(pc, color="green")

        pdata_stationary = pv.PolyData(self.stationary_vertices, point_size=self.clearance)
        sphere = pv.Sphere(radius=self.clearance)
        pc = pdata_stationary.glyph(scale=False, geom=sphere)
        p.add_mesh(pc, color="red")
        

def animate_trajectory(p: pv.Plotter, roadmap: Roadmap, start_indices, end_indices, paths):
    p.show(interactive_update=True, auto_close=False)
    start_points = np.array([roadmap.vertices[i] for i in start_indices])
    bot_spheres = [pv.Sphere(radius=2.0, center=pt) for pt in start_points]
    for sphere in bot_spheres:
        p.add_mesh(sphere)

    vert_idx = 0
    cur_step = 0
    steps = 200
    while True:
        for i, bot in enumerate(bot_spheres):
            if vert_idx + 1 < len(paths[i]):
                pt0 = roadmap.vertices[paths[i][vert_idx]]
                pt1 = roadmap.vertices[paths[i][vert_idx + 1]]
                dir = (pt1 - pt0) / float(steps)
                bot.translate(dir)
        cur_step += 1
        if cur_step == steps:
            cur_step = 0
            vert_idx += 1

        time.sleep(0.01)
        p.update()
        

@dataclass(order=True)
class CooperativeNode:
    sort_index: float = field(init=False)
    fscore: float
    gscore: float
    t_idx: int
    node_idx: int
    parent: object  # really is another CooperativeNode
    def __post_init__(self):
        self.sort_index = self.fscore


class OpenClosedList:
    def __init__(self):
        self.list = []
        self.nodes = {}

    def pop_lowest_fscore(self) -> CooperativeNode:
        node = heapq.heappop(self.list)
        # Probably not the most efficient way to do this, but we can have multiple instances of a node with multiple
        # different fscores.  We need to ignore the old high fscore nodes if we've popped a low fscore one.
        # we're basically taking this opportunity to clean up those dirty entries
        while node.node_idx not in self.nodes:
            node = heapq.heappop(self.list)
        self.nodes.pop(node.node_idx)
        return node

    def push(self, node: CooperativeNode):
        # TODO do we need to replace any existing nodes in the heap or is this ok?
        heapq.heappush(self.list, node)
        self.nodes[node.node_idx] = node

    def is_empty(self):
        return len(self.list) == 0

    """
    If there's a node in the position of passed 'node' with a lower fscore, return true
    """
    def node_with_lower_fscore(self, node):
        if node.node_idx not in self.nodes:
            return False
        if node.fscore > self.nodes[node.node_idx].fscore:
            return True

class CooperativePlanner:
    def __init__(self, roadmap: Roadmap):
        self.reservation_table = []
        self.roadmap = roadmap

    def plan(self, start_indices, end_indices):
        self.reservation_table = []
        plans = []
        # plan from start to end with no collisions.  Assume each edge takes the same amount of time to traverse. for now
        for i in range(len(start_indices)):
            plans.append(self.plan_single(start_indices[i], end_indices[i]))
        return plans

    def trace_path(self, closed_list, curnode: CooperativeNode):
        path = []
        path.append(curnode.node_idx)
        # TODO need to add curnode to reservation table?
        parent = curnode.parent
        while parent is not None:
            path.append(parent.node_idx)
            self.reservation_table[parent.t_idx].add(parent.node_idx)
            parent = parent.parent
        path.reverse()
        return path

    def plan_single(self, start_index, end_index):
        closed_list = OpenClosedList()
        open_list = OpenClosedList()
        open_list.push(CooperativeNode(0, 0, 0, start_index, None))  # f score, t index, curnode
        self.reservation_table.append(set([start_index]))

        while not open_list.is_empty():
            curnode = open_list.pop_lowest_fscore()
            curt = curnode.t_idx
            closed_list.push(curnode)
            for connected_idx in self.roadmap.edges_lookup[curnode.node_idx]:
                # make sure connected_idx is not in the reservation table for t+1
                if len(self.reservation_table) <= curt + 1:
                    self.reservation_table.append(set([]))
                elif connected_idx in self.reservation_table[curt + 1]:
                    continue 

                g = curnode.gscore + self.roadmap.node_distance(curnode.node_idx, connected_idx)
                h = self.roadmap.node_distance(connected_idx, end_index)
                fscore = g + h
                newnode = CooperativeNode(fscore, g, curnode.t_idx + 1, connected_idx, curnode)

                if connected_idx == end_index:
                    # found the goal, retrace
                    return self.trace_path(closed_list, newnode)

                if open_list.node_with_lower_fscore(newnode):
                    continue
                if closed_list.node_with_lower_fscore(newnode):
                    continue
                open_list.push(newnode)
                
            # TODO WAIT if no new nodes pushed to list 
        return None
                
                

if __name__ == "__main__":
    cyl = structures.truss_oneill_cylinder(20, 50.0, 2, 5.0)
    colliders = cyl.get_collision_objects(0.2)
    bounds = Bounds(-100., -100., -100., 100., 100., 100.)


    roadmap = Roadmap(bounds, colliders, 2.0)
    NUMBOTS = 10

    bot_positions = []
    for i in range(NUMBOTS):
        bot_positions.append(np.array([0, 0, 5.0*i]))
    bot_indices = []
    for i in range(NUMBOTS):
        # Add the bot starts before we add all the nodes
        bot_indices.append(roadmap.add_stationary_vertex(bot_positions[i]))


    goal_positions = []
    for i in range(NUMBOTS):
        goal_positions.append(np.array([50.0, 70.0, 5.0*i]))
    goal_indices = []
    for i in range(NUMBOTS):
        goal_indices.append(roadmap.add_stationary_vertex(goal_positions[i]))
    
    for i in tqdm.tqdm(range(500)):
        roadmap.add_random_vertex()
    p = pv.Plotter()

    planner = CooperativePlanner(roadmap)
    plans = planner.plan(bot_indices, goal_indices)

    #roadmap.visualize_roadmap(p, 0.2)
    structures.visualize_truss(cyl, p) 
    animate_trajectory(p, roadmap, bot_indices, goal_indices, plans) 

    print("Got plan: ", plans)

    p.show()
    
