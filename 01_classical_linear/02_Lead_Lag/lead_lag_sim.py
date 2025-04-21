import numpy as np
import matplotlib.pyplot as plt
from lead_lag import LeadLagCompensator, LeadLagPlant

# Time setup
T = 10.0
dt = 0.01

r_func = lambda t: 1.0

comp = LeadLagCompensator(z1=1, z2=5, p1=0.2, p2=10, T=T, dt=dt)
t, u = comp.simulate(r_func)

plant = LeadLagPlant(T=T, dt=dt)
t, y = plant.simulate(lambda t_val: np.interp(t_val, t, u))

plt.figure(figsize=(10, 6))
plt.plot(t, y, label='Output y(t) [Plant]')
plt.plot(t, u, label='Compensated control u(t)', linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Output")
plt.title("Step Response with Lead-Lag Compensator")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
