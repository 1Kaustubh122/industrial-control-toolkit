# PID Controller Family

Deterministic, diagonal MIMO PID (P / PI / PD / PID / PIDF) with a fixed safety chain, hooks, gain scheduling, and health telemetry. No heap allocations after `start()`.

---

## Overview & Goals

- Safe commands: stay within limits, change smoothly, and avoid shocks.
- Predictable behavior: fixed‑timestep, byte‑identical outputs for the same inputs.
- Smooth handover: switch controller/mode without output jumps (bumpless).
- Operationally robust: no runtime heap, single‑thread, no globals.

This section states intent in plain terms; subsequent sections give precise equations, constraints, and API details.

## Terminology

- `ny`: number of measured outputs (feedback channels).
- `nu`: number of command outputs (actuator channels).
- `nx`: size of optional state estimate (unused by PID core).
- `dt_ns`: fixed tick period in nanoseconds (e.g., 1’000’000 = 1 ms).
- `t_ns`: absolute time tag (nanoseconds) supplied by the caller each tick.
- `Scalar`: numeric type for math (`double` by default; `float` if configured).
- `valid_bits`: bitmask; lowest `nu` bits must be 1 when a channel’s measurement is valid.
- `SISO/MIMO`: single‑ or multi‑channel control; MIMO here is diagonal (channels independent).

## Capabilities

- **Diagonal MIMO only:** `ny == nu`. Channels are independent.
- **2-DOF structure:** β on P. γ for D is parsed but must be `0` (derivative on **measurement** only to avoid kicks; derivative setpoint weighting is not supported in this version).
- **Derivative filter:** first-order, Tustin discretized.
- **Anti-windup:** back-calculation or conditional; off mode for tests.
- **Feedforward:** constant bias (`u_ff_bias`) + dynamic FF via a pre-clamp hook.
- **Gain scheduling:** scalar variable = `y[0]`, piecewise-linear between user breakpoints (segment via binary search, then linear interpolation).
- **Safety chain (fixed order):** saturation → rate → jerk → anti-windup update.
- **Bumpless alignment:** API to align internal states to a held command.
- **Health metrics:** saturation, rate/jerk hits, AW magnitude, watchdog, etc.
- **Form:** positional PID (not velocity form).
- **Scalar:** `double` by default; `float` if configured (see `include/ictk/core/types.hpp`).
- **Channel limit:** supports up to 64 channels (`nu ≤ 64`) due to 64‑bit validity mask.

---

## Runtime Contract

- **Dims:** SISO or diagonal MIMO (`ny == nu`). `nx` is validated but unused by PID.
- **Ticking:** Fixed `dt_ns`. Caller supplies the plant time `t` each update.
- **Validity mask:** `valid_bits` must have the lowest `nu` bits set. If not, `update` returns `kPreconditionFail` and **does not** mutate controller state.
- **Memory discipline:** All buffers and safety blocks are allocated from a `MemoryArena` during `init/configure`. No allocations after `start()`.

- **Lifecycle:** Call `configure()` after `init()` and before `start()`/`update()`. Using `start()`/`update()` prior to `configure()` is invalid.
- **Bounds:** `nu ≤ 64`. Require `dt_ns > 0`.

### Units

- Signals `y`, `u`, and `r` are in user‑chosen units (SI recommended). No hidden conversions.
- Gains follow units accordingly (e.g., `Ki` in 1/s if `u` and `y` share units).

---

## Principles & Guarantees

- Determinism: no wall‑clock in control; outputs depend only on inputs, config, and `t_ns` ticks.
- Timing: strictly fixed‐timestep model; watchdog records period violations.
- Memory: all working buffers from the provided arena; no `new`/`malloc` during operation.
- Thread model: single‑thread; reentrant per instance; no globals/TLS.
- Complexity: `O(nu)` per step; no size‑dependent branching.
- ABI/portability: `noexcept` APIs; optional exceptions/RTTI can be disabled via macros.

---

## Update Order (enforced by `ControllerBase`)

1. Compute core PID: `compute_core(ctx)`.
2. `pre_clamp(out.u)` hook (optional). Use for dynamic feedforward or shaping.
3. Snapshot `u_pre` (post-hook, pre-safety).
4. Safety chain in place on a work buffer:
   - **Saturation** `[umin, umax]`
   - **Rate limit** `|Δu| ≤ du_max·dt`
   - **Jerk limit** `|Δu − Δu_prev| ≤ ddu_max·dt`  
     (current version uses uniform `rmax = du_max[0]` with `jmax = ddu_max[0]`)
5. Anti-windup update gets both `u_pre` and post-safety `u_sat`.
6. Health fields updated. `post_arbitrate(u_pre, u_sat)` hook (optional).

**Internal buffers:**
- `pre_buf_`  — saved `u_pre`
- `work_buf_` — mutable vector edited by safety steps
- `stage_buf_`— scratch to measure per-stage clip magnitude

---

## Per-Channel Math

Let `dt_s = dt_ns * 1e-9`.

**Error and weights**
```

e = β·r − y          // β in \[0,1], default 1

```

**Derivative on measurement (filtered)**
```

a1 = (2·τf − dt_s) / (2·τf + dt_s)
b  =  2            / (2·τf + dt_s)

ẏ_f[k] = b·(y[k] − y[k−1]) + a1·ẏ_f[k−1]
D      = −Kd · ẏ_f

```
`τf` may be provided directly (`tau_f`) or via `N` with `τf = 1/N`.

**Integrator (rectangle rule, Ki·dt cached per channel)**
- **Back-calculation**
```

I ← I + (Ki·dt_s)·e + Kt·(u_sat − u_pre)

```
- **Conditional**
```

I ← I + (Ki·dt_s)·e     // only when not saturated; else hold

```
- **Off** (test/debug)
```

I ← I + (Ki·dt_s)·e     // no AW protection

```

**Output before safety**
```

u_pre = Kp·e + I + D + u_ff_bias

```

---

## Gain Scheduling (optional)

- Schedule variable is `y[0]`.
- `bp[]` strictly increasing, `B ≥ 2`.
- Linearly interpolate `{Kp, Ki, Kd, β}` between `bp[i]` and `bp[i+1]`.
- γ table is accepted by the schema but must be **all zeros** in this version.
- Segment selection via binary search (`upper_bound`), then linear interpolation. No allocations.
- When the schedule variable lies outside `[bp.front(), bp.back()]`, interpolation clamps to the closest segment endpoint.

---

## Safety Chain Details

- **Saturation:** per-channel clamp to `[umin, umax]`. Provide both `umin` and `umax` to enable this stage.  
  Health: `saturation_pct`, `last_clamp_mag`.
- **Rate limit:** `|Δu| ≤ du_max·dt_s`.  
  Health: `rate_limit_hits`, `last_rate_clip_mag`.
- **Jerk limit:** `|Δu − Δu_prev| ≤ ddu_max·dt_s`.  
  Health: `jerk_limit_hits`, `last_jerk_clip_mag`.

After these, `anti_windup_update(ctx, u_pre, u_sat)` runs. Health also records:
```

aw_term_mag = Σ_i |u_sat\[i] − u_pre\[i]|

```

---

## Watchdog and Fallback

- Optional `Watchdog(dt, miss_threshold, slack)`. On trip, `health.fallback_active = true`.  
- A `FallbackPolicy` object can be configured (`safe_u`, `fb_ramp_rate`) and allocated. The base pipeline does **not** auto-apply it; use hooks if you want arbitration with a fallback vector.
- PID allocates FallbackPolicy only if configured; it is not used automatically.

---

## Health Fields (`ControllerHealth`)

- `deadline_miss_count`
- `saturation_pct`
- `rate_limit_hits`, `jerk_limit_hits`
- `fallback_active`
- `novelty_flag` (reserved)
- `aw_term_mag`
- `last_clamp_mag`, `last_rate_clip_mag`, `last_jerk_clip_mag`

---

## Bumpless Alignment

API: `align_bumpless(u_hold, r0, y0)`.

Sets internal states so the very next tick outputs ~`u_hold`:
```

e0    = β·r0 − y0
ydot0 = current filtered derivative (kept consistent)
I0    = u_hold − [ Kp·e0 − Kd·ydot0 + u_ff_bias ]
D0    = −Kd·ydot0

// Internals primed to (r0, y0), I := I0, derivative filter consistent with ydot0

````

Notes:
- `align_bumpless` prepares the next tick to hold `u_hold` under current gains and states. Use when hot‑swapping controllers or changing modes.
- For integrator preloading to a specific target output under different gains, compute `I0` using the same formula above with the new gains, then pass the appropriate `(u_hold, r0, y0)` to align.

---

## Configuration Summary (Intent + Exact)

- Gains `Kp, Ki, Kd`:
  - Intent: tune responsiveness (P), steady‑state removal (I), and damping (D).
  - Exact: per‑channel or broadcast values. `Ki` has units 1/s if `u` and `y` share units.

- Weights `β, γ`:
  - Intent: reduce overshoot on setpoint steps (β); avoid derivative kicks (γ=0).
  - Exact: `β ∈ [0,1]` (default 1). `γ` must be `0` in this version (D on measurement only).

- Derivative filter `τf` (or `N`):
  - Intent: smooth derivative to reduce noise and kicks.
  - Exact: first‑order Tustin filter with `a1, b` from `τf` or `τf = 1/N`.

- Feedforward `u_ff_bias`:
  - Intent: static bias added before safety; dynamic FF can be applied via the pre‑clamp hook.
  - Exact: per‑channel or broadcast constant.

- Saturation `[umin, umax]`:
  - Intent: hard actuator bounds.
  - Exact: provide both `umin` and `umax` to enable; per‑channel or broadcast.

- Rate limit `du_max`:
  - Intent: bound how fast commands can change.
  - Exact: per‑tick bound is `du_max * dt_s`.

- Jerk limit `ddu_max` (with uniform `du_max[0]`, `ddu_max[0]`):
  - Intent: bound how fast the rate itself can change for smooth motion.
  - Exact: per‑tick bound on Δrate is `ddu_max * dt_s`.

- Anti‑windup `{mode, Kt}`:
  - Intent: keep the integrator from drifting when clamped; unwind quickly after saturation.
  - Exact: modes `{backcalc, conditional, off}`. For back‑calc, choose `Kt > 0` and integrate `Kt·(u_sat − u_pre)` into `I`.

- Watchdog `{miss_threshold, slack}`:
  - Intent: detect missed periods; coordinate fallback.
  - Exact: when tripped, sets a health latch; application can arbitrate with a `FallbackPolicy`.

- Scheduling `{bp, *_tab}`:
  - Intent: adjust gains across operating range.
  - Exact: strictly increasing breakpoints; per‑table length equals `bp.size()`; γ table must be zeros.

---

## Configuration Schema

All entries accept either a single value (broadcast to all channels) or an array of length `nu`.

- Gains: `Kp, Ki, Kd`
- Weights: `beta` (default 1), `gamma` (must be 0)
- Derivative filter: `tau_f` or `N` (`tau_f = 1/N`)
- Feedforward: `u_ff_bias`
- Limits: `umin, umax, du_max, ddu_max`
- Anti-windup: `aw_mode ∈ {backcalc, conditional, off}`, `Kt`
- Watchdog: `miss_threshold, watchdog_slack`
- Fallback: `safe_u, fb_ramp_rate`
- Scheduling: `bp, kp_tab, ki_tab, kd_tab, beta_tab, gamma_tab`  
  (γ tab should be zeros)

Validation performed during `configure()`:
- `beta ∈ [0,1]`
- `gamma == 0`
- `bp` strictly increasing and each table length equals `bp.size()`

Additional guidance:
- If `aw_mode = backcalc`, choose `Kt > 0` (positive back‑calculation gain).
- Provide both `umin` and `umax` to enable saturation. Provide `du_max` to enable rate limiting. Provide both `du_max` and `ddu_max` to enable jerk limiting (current jerk limiter uses uniform `du_max[0]` and `ddu_max[0]`).
- Hooks must not allocate or block; they run on the real‑time path.

---

## Limits & Failure Modes

- `ny != nu` → initialization/configuration fails.
- `dt_ns ≤ 0` → invalid; filter conditioning degrades as `dt_s → 0`.
- `valid_bits` missing any of lowest `nu` bits → `update()` returns `kPreconditionFail` and does not mutate state/output.
- Only one of `umin/umax` set → saturation stage stays disabled.
- `γ ≠ 0` → configuration rejected.
- Scheduling: non‑monotonic breakpoints or mismatched table sizes → configuration rejected.

---

## Minimal Usage

```cpp
Dims d{ .ny=1, .nu=1, .nx=0 };
dt_ns dt = 1'000'000; // 1 ms
alignas(64) std::byte buf[4096];
MemoryArena arena(buf, sizeof(buf));

PIDCore pid;
ICTK_CHECK(pid.init(d, dt, arena, {}) == Status::kOK);

PIDConfig c{};
Scalar Kp[]{2.0}, Ki[]{1.0}, Kd[]{0.1};
Scalar beta[]{1.0}, gamma[]{0.0}, tf[]{0.01}, bias[]{0.0};
Scalar umin[]{-1.0}, umax[]{1.0};

c.Kp={Kp,1}; c.Ki={Ki,1}; c.Kd={Kd,1};
c.beta={beta,1}; c.gamma={gamma,1};
c.tau_f={tf,1}; c.u_ff_bias={bias,1};
c.umin={umin,1}; c.umax={umax,1};

ICTK_CHECK(pid.configure(c) == Status::kOK);
ICTK_CHECK(pid.start() == Status::kOK);

std::vector<Scalar> y(1,0), r(1,1), u(1,0);
PlantState ps{ .y={y.data(),1}, .xhat={}, .t=0, .valid_bits=0x1 };
Setpoint  sp{ .r={r.data(),1}, .preview_horizon_len=0 };
Result    res{ .u={u.data(),1}, .health={} };

ps.t += dt;
ICTK_CHECK(pid.update({ps, sp}, res) == Status::kOK);
// use res.u[0]; inspect res.health
````

---

## Building and Running

**Configure and build**

```bash
mkdir -p build
cmake -B build -S . -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DICTK_BUILD_BENCHMARKS=ON
cmake --build build -j
```

**Run tests**

```bash
ctest --test-dir build --output-on-failure
```

### Benchmarking (optional)

The benchmark runner uses wall‑clock timers and may set thread affinity / lock memory to reduce jitter. It is for profiling only; controllers remain deterministic and use caller‑provided timestamps.

---

## Benchmark Runner

Binary: `benchmarks/bench_pid_vs_baseline`

It reports three scenarios in CSV:

* `null` — empty loop timing
* `pid`  — loop + timer + PID update
* `net`  — “pid only” approximation (subtracts null loop cost from pid loop)

**Args**

```
bench_pid_vs_baseline <nu> <iters> <dt_ns> [--no-header]
```

**Example**

```bash
# append multiple runs to a CSV next to the repo
for n in 4 8 16 32 64; do
  ./build/benchmarks/bench_pid_vs_baseline "$n" 200000 1000000 --no-header >> ./benchmarks/runners/pid_bench.csv
done
```

**CSV columns**

```
label, nu, dt_ns, iters, p50, p95, p99, p999, jmin, jmax, tag1, tag2, tag3, tag4, build
```

Percentiles and jitter are in **nanoseconds**. Numbers will vary by hardware and OS.

---

## Notes / Simplifications in this version

* Diagonal MIMO only; no cross-coupling.
* Scheduling variable is fixed to `y[0]`.
* Jerk limiter uses uniform `rmax = du_max[0]`, `jmax = ddu_max[0]`.
* γ must be `0` (derivative on measurement); derivative-on-setpoint is intentionally disabled to avoid setpoint kicks.
