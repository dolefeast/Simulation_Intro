import numpy as np

class System:
    def __init__(self, 
            *particles, 
            force_constant = 8,
            origin = (0,0), dt = 1/30):
        self.particles = particles
        masses = []
        init_state = []
        for particle in particles:
            init_state.append(particle.state)
            masses.append(particle.mass)
        self.init_states = np.array(init_state)
        self.masses = np.array(masses)
        self.origin = origin
        self.time_elapsed = 0
        self.dt = dt
        self.G = force_constant
        self.state = self.init_states
    
    def get_atraction_force(self, particle1, particle2, G):
        """
        Computes the force particle2 applies on particle1
        """
        d_vector = particle1.position() - particle2.position()
        d_norm = np.linalg.norm(d_vector)
        m1 = particle1.mass
        m2 = particle2.mass
        to_return = -G * m1 * m2 * d_norm ** -3 * d_vector
        return to_return 


    def total_force(self):
        """
        Computes total force acting on every particle
        """
        G = self.G
        particles = self.particles
        state = self.state
        forces = [[0,0]]*len(particles)
        for i in range(len(particles)):
            for j in range(len(particles)):
                if j == i:
                    continue
                forces[i] += self.get_atraction_force(particles[i],
                        particles[j], G)/particles[i].mass
        return np.array(forces)
    
    def dstate_dt(self):
        state = self.state
        total_force = self.total_force()
        dydx = np.zeros_like(state)
        dydx[:,0] += state[:,2]
        dydx[:,1] += state[:,3]
        dydx[:,2] += total_force[:,0]
        dydx[:,3] += total_force[:,1]
        return dydx

    def step(self):
        dt = self.dt
        self.state = self.state + self.dstate_dt() * dt
        self.time_elapsed += dt

    def positions(self):
        state = self.state
        particles = self.particles
        position = np.array([[0, 0]]*len(particles))
        position[:,0] = position[:,0] + state[:,0]
        position[:,1] = position[:,1] + state[:,1]
        return position

    def update_states(self):
        state = self.state
        particles = self.particles
        particle_positions = self.positions()
        for i in range(len(particles)):
            particles[i].position()[0] = particle_positions[i,0]
            particles[i].position()[1] = particle_positions[i,1]

class Particle:
    """
    Particle class
    ========================================
    parameters:
        init_state = [x, y,v_x, v_y], initial position and speed
        atraction: function that dictates how 2 particles atract. 
        M = integer, field generator mass * gravitational constant
        origin = (x0, y0), 
    returns:
        time_elapsed
        state = [x, v_x, y, v_y]
        position
        dstade_dt
        step        
    """
    def __init__(self, name,
                init_state = [0, 10, 10, 0],
                m = 20):
        self.init_state = np.asarray(init_state, dtype='float')
        self.mass = m
        self.time_elapsed = 0
        self.state = self.init_state
        self.name = name
    def position(self):
        return self.state[0:2]
    def speed(self):
        return self.state[2:3]
    """ 
    def energy(self):
        m = self.satelite_mass
        M = self.force_constant
        v = np.array(self.state[1], self.state[3])
        r_norm = np.linalg.norm(self.position())
        K = 0.5 * m * np.dot(v, v)
        U = -5 * M * m / r_norm
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
   """ 
