import matplotlib.pyplot as plt
import scipy.integrate as integrate
from matplotlib.animation import FuncAnimation
import numpy as np

class Particle:
    def __init__(self,
                init_state = [0, 10, 10, 0],
                M = 600, origin = (0,0)):
        self.init_state = np.asarray(init_state, dtype='float')
        self.force_constant = M
        self.origin = origin
        self.time_elapsed = 0

        self.state = self.init_state

    def position(self):
        M = self.force_constant
        x = np.cumsum([self.origin[0], self.state[0]])
        y = np.cumsum([self.origin[1], self.state[1]])
        return (x, y)
    
    def dstate_dt(self, state):
        M = self.force_constant
        dydx = np.zeros_like(state)
        
        dydx[0] = state[1]
        dydx[2] = state[3]

        r = np.array(self.position())[:, 1]
        r_norm_squared = np.dot(r, r)
        g = -M/(r_norm_squared**1.5) * r
        dydx[1] = g[0]
        dydx[3] = g[1]
        return dydx
        
    def step(self, dt):
        dydx = self.dstate_dt(self.state)
        self.state += dydx*dt
        self.time_elapsed += dt
    """
    def step(self, dt):
        self.state = integrate.odeint(self.dstate_dt, self.state, [0, dt])[1]
        self.time_elapsed += dt
"""
    
r_x = 0
r_y = 10
v_x = 10
v_y = 0
M = 5000
particle = Particle([r_x, r_y, v_x, v_y], M)
dt = 1/30


fig = plt.figure()
ax = fig.add_subplot(111, aspect = 'equal', autoscale_on = True, xlim = (-50, 50), ylim = (-50, 50))
        
line, = ax.plot([], [], 'o-',lw = 0)
time_text = ax.text(0.02, 0.95, '', transform = ax.transAxes)

def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text

def animate(i):
    global particle, dt
    particle.step(dt)

    line.set_data(*particle.position())
    time_text.set_text('time = %.1f' % particle.time_elapsed)
    return line, time_text

from time import time
t0 = time()
animate(0)
t1 = time()
interval = 1000*dt - t1 + t0

ani = FuncAnimation(fig, animate, frames = 300,
                    interval = interval, blit = True,
                    init_func = init)
    
ani.save('double_pendulum.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
plt.show()