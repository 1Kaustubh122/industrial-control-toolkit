import numpy as np
from scipy.integrate import solve_ivp


class LeadLagPlant:
    """
    A plant (system being controlled) -> Modelled as a second order differential eqn
    """
    def __init__(self, T=10.0, dt=0.01):
        self.T = T
        self.dt = dt
        self.t = np.arange(0, T, dt)
        
    def simulate(self, u_func):
        """
        Simulating the Lead lag plant:
        G(s) = 1 / (s * (s + 2)) <- Transfer function in the laplace domain
        
        Differential Equation:
        y'' + 2y' = u(t)         <- Plants response to an input u(t)
        
        state:
        x1 = y, x2 = y'
        """
        
        def plant_ode(t, state):
            x1, x2 = state
            u_t = u_func(t)
            dx1 = x2
            dx2 = -2 * x2 + u_t
            return [dx1, dx2]
        
        sol = solve_ivp(plant_ode, [0, self.T], [0, 0], t_eval=self.t)
        return self.t, sol.y[0]

class LeadLagCompensator:
    """
    A compensator (the controller) -> designed to modify the inputs to the plant for better performance
    """
    def __init__(self, z1=1.0, z2=5.0, p1=0.2, p2=10.0, T=10.0, dt=0.01):
        self.z1 = z1
        self.z2 = z2
        self.p1 = p1
        self.p2 = p2
        self.T = T
        self.dt = dt
        self.t = np.arange(0, T, dt)
        
    def simulate(self, r_func):
        """
        Simulating the lead-lag compensator:
        G(s) = (s+z1)(s+z2) / (s+p1)(s+p2)

        Implemented as:
        u'' + (p1+p2)u' + p1*p2*u = r'' + (z1+z2)r' + z1*z2*r
        """
        def comp_ode(t, state):
            u, du = state
            r = r_func(t)
            dr = (r_func(t + 1e-5) - r_func(t)) /  1e-5
            ddr = (r_func(t + 2e-5)-2 * r_func(t+1e-5) + r_func(t)) / (1e-5**2)
            
            a0 = self.p1 * self.p2 
            b0 = self.z1 * self.z2

            a1 = self.p1 + self.p2
            b1 = self.z1 + self.z2
            
            ddu = ddr + b1 * dr + b0 * r - a1 * du - a0 * u
            return [du, ddu]
        
        sol = solve_ivp(comp_ode, [0, self.T], [0, 0], t_eval=self.t)
        
        return self.t, sol.y[0]