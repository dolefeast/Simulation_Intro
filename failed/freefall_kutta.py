import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from itertools import count

#Physical variables
F = 6000 #G * M, abreviated.
#Position 
r0 = 10#Starting distance
r_y = r0 #Updatable starting distance
r_x = 0 #Horizontal position
r_t = r_y - 35 #Earth's radius
r = np.array([r_x, r0])
r_norm = np.linalg.norm(r)


#Speed
v_x = 17 #Horizontal speed
v_y = 0 #Inicial speed
v = np.array([v_x, v_y]) 


mu = 0.05 #Air ressistance coefficient
dt = 1 #Time step

#Phyics f0unctions
def get_g(r, F = F):
    return -F * np.linalg.norm(r)**-3 * r
def get_air_ressistance(v, mu=mu):
    return mu * v
def get_acceleration(r, v, F=F):
    return get_g(r) - get_air_ressistance(v)
def update_v(r, v):
    acc = get_g(r)
    return v + acc * dt
def update_r(r, v, h = dt):
    #Runge kutta method
    k1 = update_v(r, v)
    k2 = update_v(r + 0.5 * dt, v + 0.5 * k1)
    k3 = update_v(r + 0.5 * dt, v + 0.5 * k2)
    k4 = update_v(r + 0.5 * dt, v + k3)

    r = r + h*(k1 + 2*k2 + 2*k3 + k4)/6
    
    return r

#Simulation  
cont = 0

xdata, ydata = [], []
fig, ax = plt.subplots()
plt.axis([-2*r_norm, 2*r_norm, -2*r_norm, 2*r_norm])
plt.scatter(0,0)
ln, = plt.plot([], [], 'ro')

def init():
    ax.set_xlim(-2*r_norm, 2*r_norm)
    ax.set_ylim(-2*r_norm, 2*r_norm)
    return ln, 


j = np.array([r_x, r_y])
def update(frame):
    global j
    print(j)
    j = update_r(j, v)
    xdata.append(r[0])
    ydata.append(r[1])
    ln.set_data(xdata, ydata)
    return ln,
    
def main(r = r, v = v):
    while True:
        v = update_v(r, v)
        r = update_r(r, v)
        plt.scatter(r[0], r[1], s = 1)
        plt.pause(0.05)



ani = FuncAnimation(fig, func = update, interval = 10,
                   init_func = init, blit = True)

plt.show()

