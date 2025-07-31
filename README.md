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
<div align="center">
  <img src="https://github.com/Desmondfotock28/Quadcopter/blob/main/dynamics_quadcopter.jpg?raw=true" alt="F450 Quadcopter_Dynamics" height="300"><br>
  <sub><b>Figure2:</b> Quadcopte F450 body frames and coordinate system used</sub>
</div>

The quadcopter state space is described between the earth inertial frame $E$ and body fixed frame $B$, as $`\xi = \left[\begin{array}{cccc}p_{E} & q_{E} & v_{B} & \omega_{B}\end{array}\right]^T`$ corresponding to position $`p_{E} ∈ \mathbb{R}^3`$, $`q_{E} ∈ \mathbb{R}^3`$ correspond to the Euler angles, $`v_{B} ∈ \mathbb{R}^3`$ reprsent the linear velocity and lastly  $`\omega_{B} ∈ \mathbb{R}^3`$ is the angular velocity.
 $`u =\left[T_1, T_2, T_3, T_4\right]^T`$



# PID Controller Design for F450 Quadcopter System
