import numpy as np
class Particle:
    """
    Particle class
    ========================================
    parameters:
        init_state = [x, v_x, y, v_y], initial position and speed
        M = integer, field generator mass * gravitational constant
        origin = (x0, y0), 
    returns:
        time_elapsed
        state = [x, v_x, y, v_y]
        position
        dstade_dt
        step        
    """
    def __init__(self,
                init_state = [0, 10, 10, 0],
                M = 600, m = 20,
                origin = (0,0)):
        self.init_state = np.asarray(init_state, dtype='float')
        self.force_constant = M
        self.satelite_mass = m
        self.origin = origin
        self.time_elapsed = 0

        self.state = self.init_state

    def position(self):
        M = self.force_constant
        x = self.state[0] - self.origin[0]
        y = self.state[2] - self.origin[1]
        return (x, y)

    def energy(self):
        m = self.satelite_mass
        M = self.force_constant
        v = np.array(self.state[1], self.state[3])
        r_norm = np.linalg.norm(self.position())
        K = 0.5 * m * np.dot(v, v)
        U = 5 * M * m / r_norm
        return U + K
    def dstate_dt(self, state):
        M = self.force_constant
        dydx = np.zeros_like(state)
        
        dydx[0] = state[1]
        dydx[2] = state[3]

        r = np.array(self.position())
        print(r)
        r_norm = np.linalg.norm(r)
        g = -M* r_norm ** -3 * r
        dydx[1] = g[0]
        dydx[3] = g[1]
        return dydx
        
    def step(self, dt):
        dydx = self.dstate_dt(self.state)
        self.state += dydx*dt
        self.time_elapsed += dt