from collections import namedtuple
import numpy as np
from typing import Tuple

import pyvista as pv
import fcl
import tqdm



Node = namedtuple('Node', ["position"])
Truss = namedtuple('Truss', ["node0", "node1"])


class TrussStructure:
    def __init__(self):
        self.nodes = []
        self.trusses = []
        self._node_array = np.array([[]])
        self.stale = False

    @property
    def node_array(self):
        # This isn't perfect, in case we can remove nodes,
        # should add a stale flag
        if self.stale:
            self._node_array = np.array(self.nodes)
            self.stale = False
            return self._node_array
        else:
            return self._node_array
    
    def get_collision_objects(self, radius: float):
        colliders = []
        sphere = fcl.Sphere(radius)
        for node in self.nodes:
            tf = fcl.Transform(node)
            colliders.append(fcl.CollisionObject(sphere, tf))

        for truss in self.trusses:
            node0 = self.nodes[truss.node0]
            node1 = self.nodes[truss.node1]
            _, col = cyl_from_points(node0, node1, radius)

            colliders.append(col)

        return colliders


    def add_node(self, node: np.ndarray):
        self.stale = True
        self.nodes.append(node)

    def add_truss(self, truss: Tuple[int, int]):
        self.stale = True
        self.trusses.append(Truss(truss[0], truss[1]))


def rotation_matrix_from_points(node0, node1):
    z_up = np.array([0, 0, 1.0])

    z_basis = node1 - node0
    z_basis = z_basis / np.linalg.norm(z_basis)
    x_basis = np.cross(z_basis, z_up)
    if np.linalg.norm(x_basis) < 0.001:
        x_basis = np.array([1.0, 0, 0])
    x_basis = x_basis / np.linalg.norm(x_basis)
    y_basis = np.cross(z_basis, x_basis)
    y_basis = y_basis / np.linalg.norm(y_basis)
    R = np.array([x_basis, y_basis, z_basis])
    return R

def cyl_from_points(node0, node1, radius):
    R = rotation_matrix_from_points(node0, node1)
    length = np.linalg.norm(node0 - node1)
    center_arr = (node0 + node1) / 2.0
    center = fcl.Transform(center_arr)
    cyl = fcl.Cylinder(radius, length)
    col = fcl.CollisionObject(cyl, center)
    col.setRotation(R.T)
    return cyl, col



def truss_oneill_cylinder(length: int, inner_radius: float, thickness: int, 
                          truss_length: float):
    structure = TrussStructure()
    truss_angle = np.arcsin((truss_length/2.0) / inner_radius) * 2.0
    radial_trusses = int(np.floor(2.0*np.pi / truss_angle))

    def index(i, j, k):
        return i*radial_trusses*length + j*radial_trusses + k

    for i in range(thickness):
        for j in range(length):
            for k in range(radial_trusses):
                cur_radius = inner_radius + truss_length * i
                angle = float(k) / float(radial_trusses) * 2.0 * np.pi
                x = cur_radius * np.cos(angle)
                y = cur_radius * np.sin(angle)
                z = j * truss_length
                position = np.array([x, y, z])
                structure.add_node(position)

                cur_index = index(i, j , k)
                adj_rad = index(i, j, (k + 1) % radial_trusses)
                structure.add_truss((cur_index, adj_rad))

                if j + 1 < length:
                    structure.add_truss((cur_index, index(i, j+1, k)))

                if i + 1 < thickness:
                    structure.add_truss((cur_index, index(i+1, j, k)))

    return structure

def visualize_truss(structure: TrussStructure, p: pv.Plotter, nodesize=0.1,):
    raw_points = structure.node_array
    sizes = np.ones((raw_points.shape[0])) * nodesize
    segments = []
    for pt in tqdm.tqdm(structure.trusses):
        #sph = pv.Sphere(nodesize, pt, theta_resolution=10, phi_resolution=10)
        segments.append(raw_points[pt.node0])
        segments.append(raw_points[pt.node1])
    seg_arr = np.array(segments)
    trusses = pv.line_segments_from_points(seg_arr)
    p.add_mesh(trusses)

    pdata = pv.PolyData(raw_points, point_size=nodesize)
    sphere = pv.Sphere(radius=nodesize)
    pc = pdata.glyph(scale=False, geom=sphere)

    p.add_mesh(pc)
    print("Done adding points")


if __name__ == "__main__":
    print("Starting program")
    cyl = truss_oneill_cylinder(20, 30.0, 2, 2.0)
    p = pv.Plotter()
    visualize_truss(cyl, p)
    p.show()
