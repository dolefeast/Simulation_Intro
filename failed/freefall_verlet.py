"""
x(t+dt) = 2x(t)-x(t-dt)+a(t)dt**2
"""
import numpy as np
import matplotlib.pyplot as plt

#Physical variables
F = 6000 #G * M, abreviated.
#Position 
r0 = 5#Starting distance
r_y = r0 #Updatable starting distance
r_x = 0 #Horizontal position
r_t = r_y - 35 #Earth's radius
r = np.array([r_x, r0])
r_norm = np.linalg.norm(r)


#Speed
v_x = 10 #Horizontal speed
v_y = 0 #Inicial speed
v = np.array([v_x, v_y]) 

mu = 0.05 #Air ressistance coefficient
dt = 0.01 #Time step



plt.axis([-r_norm, r_norm, -r_norm, r_norm])


def get_g(r, F = F):
    return -F * np.linalg.norm(r)**-3 * r
def get_air_ressistance(v, mu=mu):
    return mu * v
def get_acceleration(r, v, F=F):
    return get_g(r) - get_air_ressistance(v)

"""
x(t+dt) = 2x(t)-x(t-dt)+a(t)dt**2
"""
def update_r(r0, r1, h = dt):
    acc = get_g(r)
    return 2 * r1 - r0 + acc * h ** 2

def main(r0 = r, v = v, h = dt):
    r1 = r0 + v * h
    while True:
        r = update_r(r0, r1)
        r0 = r1
        r1 = r
        plt.scatter(r[0], r[1], s = 1)
        print(get_g(r))
        plt.pause(0.05)
main()
plt.show()
