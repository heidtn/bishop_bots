from odio_urdf import Group, Link, Joint, Capsule, Inertial, Visual, Sphere, \
                      Collision, Origin, Geometry, Mass, Inertia, Inertial, Parent, Child, Robot, \
                      Cylinder, Axis

import numpy as np
from scipy.spatial.transform import Rotation as R

# This pulled from here: https://gist.github.com/awesomebytes/39a4ba6c64956a1aa9bd
def get_cylinder_inertia_matrix(mass, radius, height):
    """Given mass and radius and height of a cylinder return inertia matrix.
    :return: ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz
    https://www.wolframalpha.com/input/?i=inertia+matrix+cylinder&rawformassumption=%7B%22C%22,+%22cylinder%22%7D+-%3E+%7B%22Solid%22%7D
     """
    ixx = (1.0 / 12.0) * (3.0 * radius**2 + height**2) * mass
    iyy = (1.0 / 12.0) * (3.0 * radius**2 + height**2) * mass
    izz = (1.0 / 2.0) * radius**2 * mass
    ixy = 0.0
    ixz = 0.0
    iyz = 0.0
    return {"ixx": ixx, "ixy": ixy, "ixz": ixz, "ixy": ixy, "iyy": iyy,
            "iyz": iyz, "ixz": ixz, "iyz" :iyz, "izz": izz}


# This pulled from here: https://gist.github.com/awesomebytes/39a4ba6c64956a1aa9bd
def get_sphere_inertia_matrix(mass, radius):
    """Given mass and radius of a sphere return inertia matrix.
    :return: ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz
    From https://www.wolframalpha.com/input/?i=inertia+matrix+sphere
     """
    ixx = iyy = izz = (2.0 / 5.0) * radius**2 * mass
    ixy = 0.0
    ixz = 0.0
    iyz = 0.0
    return [ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz]

def truss_link(mass, radius, height, origin, name):
    inertia = get_cylinder_inertia_matrix(mass, radius, height)

    l = Link(
             Inertial(
                 Origin(xyz=origin[:3], rpy=origin[3:]),  # TODO (HEIDT) this might be wrong way, check dox
                 Mass(value=mass),
                 Inertia(**inertia)
             ),
             Visual(
                 Origin(xyz=origin[:3], rpy=origin[3:]),
                 Geometry(Cylinder(radius=radius, length=height))
             ),
             Collision(
                 Origin(xyz=origin[:3], rpy=origin[3:]),
                 Geometry(Cylinder(radius=radius, length=height))
             ),
             name=name,
        )
    return l

def node_link(mass, radius, origin, name):
    l = Link(
             Inertial(
                 Origin(origin),  # TODO (HEIDT) this might be wrong way, check dox
                 Mass(value=mass),
                 Inertia(get_sphere_inertia_matrix(mass, radius))
             ),
             Visual(
                 Origin(origin),
                 Geometry(Capsule(radius=radius))
             ),
             Collision(
                 Origin(origin),
                 Geometry(Capsule(radius=radius))
             ),
             name=name,
        )
    return l
    

def create_flats(mass, radius, height, width, count, start_origin, member_length=3.0):
    links = []
    joints = []
    for c in range(count):
        if c == 0:
            x_off = np.array([c*member_length + member_length/2.0, 0, 0])
            position = start_origin + x_off
            origin = np.array([*position, 0., 0., 0.])
            links.append(truss_link(mass, radius, height, origin, f"truss_{c}"))
        elif c < width:
            x_off = np.array([c*member_length + member_length/2.0, 0, 0])
            position = start_origin + x_off
            origin = np.array([*position, 0., 0., 0.])
            links.append(truss_link(mass, radius, height, origin, f"truss_{c}"))

        else:
            if c == width:
                pass
            else:
                pass

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

def cyl_link_from_points(point1, point2, mass, radius, name):
    origin = (point1 + point2) / 2.0
    length = np.linalg.norm(point2 - point1)
    rot_mat = rotation_matrix_from_points(point1, point2)
    r = R.from_matrix(rot_mat)
    rpy = r.as_euler('xyz')
    full_origin = [*origin, *rpy]
    return truss_link(mass, radius, length, full_origin, name), rpy

def make_joint(parent, child, name, rpy):
    ret = Joint(Parent(link=parent),
                Child(link=child),
                type="fixed",
                name=name)
    return ret

def truss_oneill_cylinder(length: int, inner_radius: float, thickness: int, 
                          truss_length: float, link_mass, truss_radius):
    #structure = TrussStructure()
    links = []
    joints = []
    node_points = []
    truss_indices = []

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
                #structure.add_node(position)
                cur_index = index(i, j , k)
                #node = node_link(node_mass, truss_radius, position, f"sphere_{cur_index}")
                #links.append(node)
                node_points.append(position)

                adj_rad = index(i, j, (k + 1) % radial_trusses)
                #structure.add_truss((cur_index, adj_rad))
                truss_indices.append((cur_index, adj_rad))
                #truss = cyl_link_from_points()

                if j + 1 < length:
                    #structure.add_truss((cur_index, index(i, j+1, k)))
                    truss_indices.append((cur_index, index(i, j+1, k)))

                if i + 1 < thickness:
                    #structure.add_truss((cur_index, index(i+1, j, k)))
                    truss_indices.append((cur_index, index(i+1, j, k)))

    base_points = truss_indices[0]
    p1 = node_points[base_points[0]]
    p2 = node_points[base_points[1]]
    base_link, rpy = cyl_link_from_points(p1, p2, link_mass, truss_radius, "base_link")
    links.append(base_link)
    for i, truss in enumerate(truss_indices[1:]):
        p1 = node_points[truss[0]]
        p2 = node_points[truss[1]]
        link, rpy = cyl_link_from_points(p1, p2, link_mass, truss_radius, f"truss_{i}")
        joint = make_joint("base_link", f"truss_{i}", f"truss_{i}_joint", rpy) 
        links.append(link)
        joints.append(joint)
    
    return Robot(*links, *joints, name="station")

if __name__ == "__main__":
    print(truss_oneill_cylinder(2, 10.0, 1, 3.0, 5.0, 0.05))