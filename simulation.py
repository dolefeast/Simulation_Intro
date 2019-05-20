import matplotlib.pyplot as plt
import scipy.integrate as integrate
from matplotlib.animation import FuncAnimation
import numpy as np
from system import Particle, System
from pprint import pprint


particle1 = Particle()
particle2 = Particle([2, 5, 7, 9])

system = System(particle1, particle2)

print(system.total_total_force()))


"""
fig = plt.figure()
ax = fig.add_subplot(111, aspect = 'equal', autoscale_on = True, xlim = (-50, 50), ylim = (-50, 50))
        
line, = ax.plot([], [], 'o-',lw = 0)
time_text = ax.text(0.02, 0.95, '', transform = ax.transAxes)
energy_text = ax.text(0.02, 0.90, '', transform=ax.transAxes)

def init():
    line.set_data([], [])
    time_text.set_text('')
    energy_text.set_text('')
    return line, time_text, energy_text
plt.scatter(0,0)
def animate(i):
    global particle, dt
    particle.step(dt)
    line.set_data(*particle.position())
    time_text.set_text('time = %.1f' % particle.time_elapsed)
    energy_text.set_text('energy = %.3f J' % particle.energy())
    return line, time_text, energy_text

from time import time
t0 = time()
animate(0)
t1 = time()
interval = 1000*dt - t1 + t0

ani = FuncAnimation(fig, animate, frames = 300,
                    interval = interval, blit = True,
                    init_func = init)

plt.show()"""
