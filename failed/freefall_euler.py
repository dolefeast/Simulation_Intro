import numpy as np
import matplotlib.pyplot as plt

#Physical variables
F = 600 #G * M, abreviated.
#Position 
r0 = 4#Starting distance
r_y = r0 #Updatable starting distance
r_x = 0 #Horizontal position
r_t = r_y - 35 #Earth's radius
r = np.array([r_x, r0])
r_norm = np.linalg.norm(r)


#Speed
v_x = 7 #Horizontal speed
v_y = -2 #Inicial speed
v = np.array([v_x, v_y]) 


mu = 0.0 #Air ressistance coefficient
dt = 0.01 #Time step

plt.axis([-r_norm, r_norm, -r_norm, r_norm])
def get_g(r, F = F):
    return -F * np.linalg.norm(r)**-3 * r
def get_air_ressistance(v, mu=mu):
    return mu * v
def get_acceleration(r, v, F=F):
    return get_g(r) - get_air_ressistance(v)
def main(r = r, v = v):
    while True:
        acc = get_acceleration(r, v)
        v = v + acc * dt
        r = r + v * dt
        plt.scatter(r[0], r[1], s = 1)
        plt.pause(0.05)
main()
plt.show()
