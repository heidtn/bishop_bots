import numpy as np
import matplotlib.pyplot as plt
from gekko import GEKKO
from mpl_toolkits import mplot3d


"""
Have a spacecraft at 0,0,0 with fuly unidirectional control.

It is attempting to get to 100, 100, 100.

It must avoid spherical asteroids on the way there and use the least amount
of fuel.
"""

# create GEKKO model
m = GEKKO()

# scale 0-1 time with tf
m.time = np.linspace(0,1,101)

# options
m.options.NODES = 6
m.options.SOLVER = 3
m.options.IMODE = 6
m.options.MAX_ITER = 500
m.options.MV_TYPE = 0
m.options.DIAGLEVEL = 0

m.Var()

# final time
tf = m.FV(value=1.0,lb=0.1,ub=100)
tf.STATUS = 1

# force
#u = m.MV(value=0,lb=-1.1,ub=1.1)
#u.STATUS = 1
#u.DCOST = 1e-5
u = m.Array(m.MV, (3))
for j in u:
    j.value = 0.0
    j.lower = -10.0
    j.upper = 10.0
    j.STATUS = 1
    j.DCOST = 1e-5


# variables
s = m.Array(m.Var, (3))
for j in s:
    j.value = 0.0

v = m.Array(m.Var, (3))
for j in v:
    j.value = 0.0

mass = m.Var(value=1,lb=0.2)

for i in range(3):
    m.Equation(s[i].dt()==tf*v[i])

for i in range(3):
    m.Equation(mass*v[i].dt()==tf*u[i])

m.Equation(mass.dt()==tf*(-0.01*m.sqrt(u[0]**2.0 + u[1]**2.0 + u[2]**2.0)))

# Avoid the sphere at knot points
#from mpl_toolkits import mplot3d
#(s[0] - sp[0])**2.0
m.Equation(m.sqrt(m.sum((s - np.array([50., 50., 50.0]))**2.0)) >= 10.0)

# specify endpoint conditions
for i in range(3):
    m.fix(s[i], pos=len(m.time)-1,val=100.0)
    m.fix(v[i], pos=len(m.time)-1,val=0.0)


# minimize final time
m.Obj(tf)

# Optimize launch
m.solve()

print('Optimal Solution (final time): ' + str(tf.value[0]))

# scaled time
ts = m.time * tf.value[0]

# plot results
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(s[0], s[1], s[2])

#ax = plt.axes(projection='3d')
#ax.plot3D(v[0], v[1], v[2])


#plt.figure(1)
#plt.subplot(4,1,1)
#plt.plot(ts,s.value,'r-',linewidth=2)
#plt.ylabel('Position')
#plt.legend(['s (Position)'])
#
#plt.subplot(4,1,2)
#plt.plot(ts,v.value,'b-',linewidth=2)
#plt.ylabel('Velocity')
#plt.legend(['v (Velocity)'])
#
#plt.subplot(4,1,3)
#plt.plot(ts,mass.value,'k-',linewidth=2)
#plt.ylabel('Mass')
#plt.legend(['m (Mass)'])
#
#plt.subplot(4,1,4)
#plt.plot(ts,u.value,'g-',linewidth=2)
#plt.ylabel('Force')
#plt.legend(['u (Force)'])
#
#plt.xlabel('Time')
plt.show()