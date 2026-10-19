# Multimode Control Barrier Function for Simulink

This repository contains a MATLAB/Simulink implementation of a multimode
control barrier function (CBF), along with the Simulink model used to
generate ROS code and a collection of simulation test cases.

## How the CBF works

A control barrier function maintains system safety by turning a safety
requirement into a constraint on the control input. At each simulation step,
the CBF evaluates the current system state, the surrounding traffic
conditions, and the nominal control command. It then determines whether the
nominal command preserves the required safety margin and applies the
appropriate constraint when it does not.

This implementation is **multimode**: the active safety constraint changes
with the operating mode of the system. Mode transitions allow the CBF to
represent different traffic or vehicle behaviors while preserving the safety
requirement across those transitions. The resulting CBF subsystem can be
inserted into a larger Simulink model between the nominal controller and the
vehicle dynamics, where it provides a safety-aware control command for
simulation or ROS deployment.

The state $x_n$ of the system contains the space gap $s_n$, the ego velocity
$v_n$, and the relative velocity $\Delta v_n$.
The constant values used by the multi-mode CBF are:

| Constant | Value |
| --- | ---: |
| $\tau$ | $2.0\ \mathrm{s}$ |
| $k$ | $0.1$ |
| $d_1^{\min}$ | $15.0\ \mathrm{m}$ |
| $d_2^{\min}$ | $5.0\ \mathrm{m}$ |
| $k_d$ | $3.0$ |
| $v_{\mathrm{slow}}$ | $5.0\ \mathrm{m/s}$ |
| $v_{\mathrm{fast}}$ | $7.5\ \mathrm{m/s}$ |
| $\epsilon$ | $0.1\ \mathrm{m}$ |
| $\tau_{1,2}$ | $3.0\ \mathrm{s}$ |
| $u_{\min}$ | $-3.0\ \mathrm{m/s^2}$ |
| $u_1^{\max}$ | $1.5\ \mathrm{m/s^2}$ |
| $u_2^{\max}$ | $1.0\ \mathrm{m/s^2}$ |
| $u_3^{\max}$ | $0.5\ \mathrm{m/s^2}$ |

In all modes, the CBF command acceleration is calculated as:

$$
u_{\mathrm{cbf}}(x_n) = \frac{1}{\tau}
\left(k\left(s_n - \left(\tau v_n + d_q^{\min}\right)\right)
+ \Delta v_n\right)
$$

The command acceleration is then:
$$
u_{\text{cmd}}(x_n) = \min \{ u_{\text{cbf}}(x_n), k_{\text{nom}}(x_n) \}
$$

In the third mode, the minimum distance is calculated as:

$$
d_3^{\min}(v_n) =
\frac{\left(d_1^{\min} - d_2^{\min}\right)
\exp\left(k_d\left(v_n -
\frac{v_{\mathrm{slow}} + v_{\mathrm{fast}}}{2}\right)\right)}
{1 + \exp\left(k_d\left(v_n -
\frac{v_{\mathrm{slow}} + v_{\mathrm{fast}}}{2}\right)\right)}
+ d_2^{\min}
$$

The mode-transition logic is summarized below. The three operating modes are
the full-range mode ($q = 1$), the stop-and-go mode ($q = 2$), and the
transition mode ($q = 3$). The transition guards use the paper's notation:
$s_n$ is the measured spacing, $v_n$ is the measured speed, $v_{\mathrm{slow}}$
and $v_{\mathrm{fast}}$ are speed thresholds, $d_1^{\min}$ and
$d_2^{\min}$ are minimum-distance terms, $\tau$ is the time-headway
parameter, and $\epsilon$ is the safety margin.

| Current mode | Next mode | Transition condition |
| --- | --- | --- |
| Full range ($q = 1$) | Stop-and-go ($q = 2$) | $s_n \ge \tau v_n + d_1^{\min} - \epsilon$ and $\operatorname{duration}(v_n \le v_{\mathrm{slow}}) \ge \tau_{1,2}$ |
| Stop-and-go ($q = 2$) | Transition ($q = 3$) | $s_n \ge \tau v_n + d_2^{\min} - \epsilon$ and $v_n \ge v_{\mathrm{fast}}$ |
| Transition ($q = 3$) | Stop-and-go ($q = 2$) | $v_n \le v_{\mathrm{slow}}$ |
| Transition ($q = 3$) | Full range ($q = 1$) | $s_n \ge \tau v_n + d_1^{\min} - \epsilon$ and $v_n \ge v_{\mathrm{slow}}$ |

At every step, the active mode selects the corresponding CBF constraint. The
vehicle or plant dynamics then provide the updated state and traffic
measurements used to evaluate the next transition.

## Repository structure

```text
.
├── multimodecbf_subsystem.slx   # CBF subsystem
├── multimodecbf.slx             # Top-level model and ROS code-generation entry point
├── startup.m                    # Adds the repository and its subdirectories to the MATLAB path
├── test/
│   ├── case_*.slx               # Simulation test cases
│   └── test_resources/          # Reusable subsystem models used by the tests
```

### Main models

- **`multimodecbf_subsystem.slx`** defines the multimode CBF. Open this model
  to inspect or modify the barrier-function subsystem.
- **`multimodecbf.slx`** is the top-level model used to integrate the CBF and
  generate ROS code. Use the model's configured Simulink build/code-generation
  workflow when producing ROS output.

### Startup

- **`startup.m`** initializes the MATLAB environment by recursively adding the
  repository to the MATLAB path:

  ```matlab
  addpath(genpath('.'))
  ```

### Tests

The **`test/`** directory contains the simulation test suite. Each
`case_*.slx` file represents a scenario used to exercise the CBF and the
surrounding vehicle/control models. The `test/test_resources/` directory
contains reusable subsystem models and supporting Simulink models consumed
by those test cases.

Test case names describe the scenario being exercised, for example:

- `case_01_fast_steady.slx` — fast, steady-state behavior
- `case_04_slow_to_fast.slx` — transition from slow to fast behavior
- `case_09_30_to_2_to_0.slx` — a speed profile that reaches a stop
- `case_13_hard_brake.slx` — hard-braking behavior
- `case_19_all_state_oscillation.slx` — oscillation across all states

## Getting started

### Requirements

You will need:

- MATLAB
- Simulink
- The MATLAB/Simulink products required by the models and their configured
  ROS code-generation workflow

The exact MATLAB release and toolbox versions should match the environment in
which the models were created and tested.

### Initialize MATLAB

From the repository root, run:

```matlab
startup
```

Alternatively, execute `startup.m` from the MATLAB Current Folder browser.
Running the script first ensures that the top-level models and all test
resources are available on the MATLAB path.

### Inspect or simulate the CBF

1. Run `startup`.
2. Open `multimodecbf_subsystem.slx` to inspect the CBF implementation, or
   open one of the models in `test/` to run a scenario.
3. Configure the model's normal Simulink simulation settings as needed.
4. Run the simulation and inspect the model outputs and scopes/logs provided by
   the selected model.

### Generate ROS code

1. Run `startup`.
2. Open `multimodecbf.slx`.
3. Confirm that the model's ROS/code-generation configuration matches your
   target environment.
4. Use the Simulink build or code-generation command configured for the model.

## Citation

To cite the usage of this repository, use the following:

Kate Sanborn, George Gunter, Daniel B. Work, Ricardo G. Sanfelice, Jonathan Sprinkle. "Traffic Density with Multi-Mode Control Barrier Functions at Scale." in 2026 IEEE International Conference on Intelligent Transportation Systems (ITSC), (in press) 2026.

or using bibtex:

```
@inproceedings{sanborn2026single,
  author    = {Sanborn, Kate and Gunter, George and Work, Daniel B. and Sanfelice, Ricardo G. and Sprinkle, Jonathan},
  title      = {Traffic Density with Multi-Mode Control Barrier Functions at Scale},
  booktitle  = {2026 IEEE International Conference on Intelligent Transportation Systems (ITSC)},
  year       = {2026},
  note       = {in press}
}
```

---
