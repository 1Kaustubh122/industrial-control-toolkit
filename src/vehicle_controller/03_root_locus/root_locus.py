import math
import numpy as np
from scipy.optimize import root_scalar

class RootLocus:
    def __init__(self, poles, zeros):
        self.poles = poles
        self.zeros = zeros
        self.n = len(self.poles) 
        self.m = len(self.zeros)
        # self.asymptotes = len(poles) - len(zeros)
    
    def calc_asympote_centroid(self):
        return (sum(self.poles) - sum(self.zeros)) / (self.n - self.m)
    
    def calc_asympote_angles(self):
        angles = []
        for q in range(self.n - self.m):
            theta = (2 * q + 1) * math.pi / (self.n - self.m)
            angles.append(theta)
        return angles
    
    def is_on_real_axis(self, point):
        if not np.isreal(point):
            return False
        
        count = 0
        for p in self.poles:
            if np.isreal(p) and p.real > point.real:
                count += 1
        for z in self.zeros:
            if np.isreal(z) and z.real > point.real:
                count += 1
                
        return count % 2 == 1
    
    def G(self, s):
        num = np.prod([s - z for z in self.zeros]) if self.zeros else 1
        den = np.prod([s - p for p in self.poles])
        return num/den
    
    def K(self, s):
        return -1 / self.G(s)
    
    def dk_ds(self, s, h=1e-5):
        return (self.K(s+h) - self.K(s-h)) / (2*h)
    
    def breakaway_points(self, search_range=(-20, 5), num_points=1000):
        points = []
        s_vals = np.linspace(search_range[0], search_range[1], num_points)
        
        for i in range(len(s_vals) - 1):
            a, b = s_vals[i], s_vals[i+1]
            
            try:
                if np.sign(self.dk_ds(a)) != np.sign(self.dk_ds(b)):
                    sol = root_scalar(self.dk_ds, bracket=(a, b), method='brentq')
                    if sol.converged:
                        root = sol.root
                        if self.is_on_real_axis(root):
                            points.append(round(root, 5))
            except:
                continue
            
        return sorted(set(points))
    
    def angle_of_departure(self, pole):
        other_poles = [p for p in self.poles if p != pole]
        sum_pole_angles = sum(np.angle(pole - p, deg=True) for p in other_poles)
        sum_zero_angles = sum(np.angle(pole - z, deg=True) for z in self.zeros)
        return 180 - (sum_pole_angles - sum_zero_angles)
    
    def satisfies_angle_conditions(self, s, tolerance=5):
        s = complex(s)
        angle = sum(np.angle(s - z, deg=True) for z in self.zeros)
        angle -= sum(np.angle(s - p, deg=True) for p in self.poles)
        angle %= 360
        return abs(angle - 180) <= tolerance or abs(angle - 540) <= tolerance
    
    def scan_complex_plane(self, xlimit=(-10, 10), ylimit=(-10, 10), resol=500):
        real_vals = np.linspace(xlimit[0], xlimit[1], resol)
        imag_vals = np.linspace(ylimit[0], ylimit[1], resol)
        locus_points = []

        for i in real_vals:
            for k in imag_vals:
                s = complex(i, k)
                if self.satisfies_angle_conditions(s, tolerance=5):
                    locus_points.append(s)

        return locus_points