# Industrial Control Toolkit (ICTK)

**A portable, deterministic, production-grade control library for real-world automation.**

ICTK brings together proven control theory methods under a single, stable C++20 API.  
It is designed for systems that must run **unchanged** across simulation, industrial PCs, embedded controllers, and PLCs — from lab prototypes to factory-floor deployments.

---

## Why ICTK exists

Industrial autonomy projects often end up juggling multiple control codebases:  
one for simulation, another for ROS2, another for embedded, and a pile of untested MATLAB scripts in between.  
That makes hot-swapping controllers, adding safety layers, or benchmarking new algorithms slow and error-prone.

ICTK fixes that with:

- **One deterministic runtime**: same behavior, same results, same timing everywhere.
- **Unified controller API**: swap PID ↔ LQR ↔ MPC ↔ robust/adaptive controllers without rewriting integration code.
- **Built-in safety**: saturation, rate/jerk limits, watchdogs, interlocks, bumpless transfer, and hierarchical fallback.
- **KPI-driven merges**: every algorithm is benchmarked and accepted only if it beats a baseline in defined metrics.
- **Portability by design**: static/shared libs, C ABI option, embedded/PLC build profiles, compile-time traits.

---

## What ICTK can do

- **PID family** — P, PI, PD, PID, PIDF, 2-DOF, IMC-PID, gain scheduling, cascade, anti-windup, feedforward.
- **State-space** — LQR, LQI, pole placement, static output feedback, LQG (controller side), decoupling.
- **MPC** — Linear MPC (SS/ARX), Reference Governor, Tube MPC, NMPC (SQP/IPM), Economic MPC.
- **Robust/adaptive** — H∞ state-feedback, ADRC, SMC, DOBC + feedforward, Smith, IMC, Repetitive, ILC, MRAC.
- **Shaping/plant tools** — lead/lag, notch design, loop shaping, model fitting, filters, scaling, steady-state solvers.
- **Optimization core** — deterministic QP/IPM solver, warm-start, factorization cache, compile-time dims.

All with the same controller interface, safety contracts, and deterministic timing guarantees.

---

## Getting started

Build and run on any target with a C++20 compiler:

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
````

Then try an example, e.g. PID position control:

```bash
./examples/pid_position_control
```

See [`docs/Architecture.md`](docs/Architecture.md) for the full module layout and API contracts.

---

## Repository structure

* **include/** — public headers, stable API.
* **src/** — implementations.
* **control/** — all controller families.
* **safety/** — safety wrappers and limits.
* **models/** — plant models, fitters, filters.
* **traj/** — trajectory generators.
* **opt/** — optimization core.
* **tools/** — CLI utilities (autotune, fitters, KPI reports).
* **examples/** — runnable demos.
* **tests/** — unit, property, fuzz tests.
* **benchmarks/** — reproducible performance and KPI baselines.
* **docs/** — architecture, API reference, tuning guides.



## License

MIT — see [LICENSE](LICENSE).


## Status

🚧 **Active development** — v1.0.0 planned with full API freeze and benchmark baselines.

