import fcl
import multiagent_planning
import structures
import numpy as np


p0 = np.array([0, -5, -10])
p1 = np.array([0, 5, 10])
s0 = np.array([0, -2.5, -10])

cyl = structures.cyl_from_points(p0, p1, 1.0)
sphere = fcl.Sphere(1.0)
tf = fcl.Transform(s0)
sph = fcl.CollisionObject(sphere, tf)

request = fcl.CollisionRequest()
result = fcl.CollisionResult()

ret = fcl.collide(cyl, sph, request, result)

print(result.is_collision)
