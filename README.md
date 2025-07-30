# Nonlinear Model Predictive Control Formulation for F450 Quadcopter

## Install Acados
To build Acados from source, see instructions [here](https://docs.acados.org/python_interface/index.html) or as follows:

Clone acados and its submodules by running:
```
$ git clone https://github.com/acados/acados.git
$ cd acados
$ git submodule update --recursive --init
```

Install acados as follows:

```
$ mkdir -p build
$ cd build
$ cmake -DACADOS_WITH_QPOASES=ON ..
$ make install -j4
```

Install acados_template Python package:
```
$ cd acados
$ pip install -e interfaces/acados_template
```
***Note:*** The ```<acados_root>``` is the full path from ```/home/```.

Add two paths below to ```~/.bashrc``` in order to add the compiled shared libraries ```libacados.so```, ```libblasfeo.so```, ```libhpipm.so``` to ```LD_LIBRARY_PATH``` (default path is ```<acados_root/lib>```):

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
export ACADOS_SOURCE_DIR="<acados_root>"
```
## Quadcopter
<div align="center">
  <img src="https://github.com/Desmondfotock28/Quadcopter/blob/main/model.PNG?raw=true" alt="F450 Quadcopter 3d Model" height="300"><br>
  <sub><b>Figure1:</b> F450 Quadcopter 3D Model</sub>
</div>

The F450 quadcopter is a versatile platform well-suited for both control system design and vision-based navigation research. In this project, we explore advanced control strategies specifically Nonlinear Model Predictive Control (NMPC) and PID controllers to enable precise trajectory tracking under dynamic flight conditions. Building upon this control foundation, we further integrate visual SLAM (Simultaneous Localization and Mapping) using an Intel RealSense camera mounted on the drone. This fusion of control and perception aims to enable the quadcopter to navigate autonomously in GPS-denied environments, with potential applications in inspection, search and rescue, and exploration. The project combines simulation and real-world experimentation to validate the effectiveness of the proposed control and perception pipeline.

## Dynamics
The full explanation of the quadrotor dynamics is presented in [here](https://rpg.ifi.uzh.ch/docs/ScienceRobotics21_Foehn.pdf).

The quadcopter state space is described between the inertial frame $I$ and body frame $B$, as $`\xi = \left[\begin{array}{cccc}p_{IB} & q_{IB} & v_{IB} & \omega_{B}\end{array}\right]^T`$ corresponding to position $`p_{IB} ∈ \mathbb{R}^3`$, unit quaternion rotation on the rotation group $`q_{IB} \in \mathbb{SO}(3)`$ given $`\left\Vert q_{IB}\right\Vert = 1`$, velocity $`v_{IB} \in \mathbb{R}^3`$, and bodyrate $`\omega_B \in \mathbb{R}^3`$. The input modality is on the level of collective thrust $`T_B = \left[\begin{array}{ccc}0 & 0 & T_{Bz} \end{array}\right]^T`$ and body torque $`\tau_B`$ . From here on we drop the frame indices since they are consistent throughout the description. The dynamic equations are follows:
```math
\dot{p}=v,\\
\dot{q}= \dfrac{1}{2}\Lambda(q)\left[\begin{array}{l}0\\\omega\end{array}\right], \\
\dot{v}=g+\dfrac{1}{m}R(q)T,\\
\dot{\omega}=J^{-1}(\tau-\omega\times J\omega)
```
where $`\Lambda`$ represents a quaternion multiplication, $`R(q)`$ the quaternion rotation, $m$ the quadrotor’s mass, and $`J`$ its inertia.

The input space given by $`T`$ and $`\tau`$ is further decomposed into the single rotor thrusts $`u =\left[T_1, T_2, T_3, T_4\right]^T`$, where $`T_i`$ is the thrust at rotor $`i \in \{1, 2, 3, 4\}`$
```math
T=\left[\begin{array}{c}0\\0\\\sum{T_i}\end{array}\right]
```
For $`\times`$ configuration quadrotor model
```math
\tau=\left[\begin{array}{c}l/\sqrt{2}(T_1-T_2-T_3+T_4)\\
                           l/\sqrt{2}(-T_1-T_2+T_3+T_4)\\
                           c_\tau(-T_1+T_2-T_3+T_4)\end{array}\right]
```
For $`+`$ configuration quadrotor model
```math
\tau=\left[\begin{array}{c}l(T_1-T_3)\\
                           l(T_2-T_4)\\
                           c_\tau(-T_1+T_2-T_3+T_4)\end{array}\right]
```
with the quadrotor’s arm length $l$ and the rotor’s torque constant $`c_\tau`$. The quadrotor’s actuators limit the applicable thrust for each rotor, effectively constraining $`T_i`$ as:
```math
0\leq T_{min} \leq T_i \leq T_{max}
```


# PID Controller Design for F450 Quadcopter System
