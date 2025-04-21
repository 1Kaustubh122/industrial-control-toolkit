import numpy as np
import matplotlib.pyplot as plt
from root_locus import RootLocus

poles = [0, -2, -4]
zeros = [-1]

ro_lo = RootLocus(poles, zeros)

centroid = ro_lo.calc_asympote_centroid()
angles = ro_lo.calc_asympote_angles()

bap = ro_lo.breakaway_points()

locus = ro_lo.scan_complex_plane(xlimit=(-10, 5), ylimit=(-10, 10), resol=300)

fig, ax = plt.subplots(figsize=(10, 8))
ax.axhline(0, color='black', lw=1)
ax.axvline(0, color='black', lw=1)

ax.plot([p.real for p in poles], [p.imag for p in poles], 'rx', markersize=10, label='Poles')
ax.plot([z.real for z in zeros], [z.imag for z in zeros], 'go', markersize=10, label='Zeros')

for angle in angles:
    x = np.linspace(-10, 10, 100)
    y = np.tan(angle) * (x - centroid.real)
    ax.plot(x, y, linestyle='--', color='gray', alpha=0.5)

for b in bap:
    ax.plot(b, 0, 'ko', markersize=6, label='Breakaway' if b == bap[0] else '')

if locus:
    ax.plot([pt.real for pt in locus], [pt.imag for pt in locus], 'b.', markersize=1, label='Root Locus')

ax.legend()
ax.grid(True)
ax.set_title("Root Locus Plot")
plt.xlabel("Real Axis")
plt.ylabel("Imaginary Axis")
plt.tight_layout()
plt.show()
