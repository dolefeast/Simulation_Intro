import matplotlib.pyplot as plt
import scipy.integrate as integrate
from matplotlib.animation import FuncAnimation
import numpy as np
from system import Particle, System
from pprint import pprint

dt = 1/30
particle1 = Particle('particle1', m = 5000)
particle2 = Particle('particle2', [20, 5, -9, 0], m=50)
particle3 = Particle('particle3', [-20, 1, 5, 6], m = 70)

system = System(particle1, particle2, particle3, dt = dt)

fig = plt.figure()
ax = fig.add_subplot(111, aspect = 'equal', autoscale_on = True, xlim = (-50, 50), ylim = (-50, 50))
        
line1, = ax.plot([], [], 'o-',lw = 0)
line2, = ax.plot([], [], 'o-',lw = 0)
line3, = ax.plot([], [], 'o-',lw = 0)
time_text = ax.text(0.02, 0.95, '', transform = ax.transAxes)

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    time_text.set_text('')
    return line1, line2, line3, time_text

def animate(i):
    global particle, dt
    system.step()
    line1.set_data(*particle1.position())
    line2.set_data(*particle2.position())
    line3.set_data(*particle3.position())
    system.update_states()
    time_text.set_text('time = %.1f' % system.time_elapsed)
    return line1, line2, line3, time_text

from time import time
t0 = time()
animate(0)
t1 = time()
interval = 1000*dt - t1 + t0

ani = FuncAnimation(fig, animate, frames = 300,
                    interval = interval, blit = True,
                    init_func = init)
"""ani.save('3-body.mp4')
"""
plt.show()
