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

## Mathematical model of quadcopter
<div align="center">
  <img src="https://github.com/Desmondfotock28/Quadcopter/blob/main/dynamic_model.jpg?raw=true" alt="F450 Quadcopter_Dynamics" height="300"><br>
  <sub><b>Figure2:</b> The inertial and body frames of a quadcopter</sub>
</div>
The quadcopter structure is presented in Figure  including the corresponding angular velocities, torques and forces created by the four rotors (numbered from 1 to 4).

The absolute linear position of the quadcopter is defined in the inertial frame x,y,z axes with $\mathbf{\xi}$. The attitude, i.e. the angular position, is defined in the inertial frame with three Euler angles $\eta$. Vector \boldsymbol{q} contains the linear and angular position vectors
# PID Controller Design for F450 Quadcopter System
