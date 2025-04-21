import math

class RootLocus:
    def __init__(self, poles, zeros):
        self.poles = poles
        self.zeros = zeros
        self.n = len(self.poles) 
        self.m = len(zeros)
        # self.asymptotes = len(poles) - 
        
    
    def calc_sigmoid(self):
        return (sum(self.poles) - sum(self.zeros)) / (self.n - self.m)


print(type(math.pi))