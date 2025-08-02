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

The absolute linear position of the quadcopter is defined in the inertial frame x,y,z axes with $\mathbf{\xi}$. The attitude, i.e. the angular position, is defined in the inertial frame with three Euler angles $\eta$. Vector $\boldsymbol{q}$ contains the linear and angular position vectors.
```math
\xi = [x, y , z]^T ;  \eta = [\phi, \theta, \psi]^T ;  q = [\xi , \eta]^T
```
### Newton-Euler equations
The quadcopter is assumed to be rigid body and thus Newton-Euler equations can be used to describe its dynamics. In the body frame, the force required for the acceleration of mass $m \dot{V}_B$ and the centrifugal force $\nu \times (mV_B)$ are equal to the gravity $R^T G$ and the total thrust of the rotos $T_B$
```math
m\dot{V}_B + \nu \times (mV_B) =\ R^TG + T_B
```
In the inertial frame, the centrifugal force is nullified. Thus, only the gravitational force and the magnitude and direction of the thrust are contributing in the acceleration of the quadcopter


```math
m\ddot{\boldsymbol{\xi}} = \mathbf{G} + \mathbf{R} \mathbf{T}_B
```
```math
\begin{bmatrix}
\ddot{x} \\
\ddot{y} \\
\ddot{z}
\end{bmatrix}
= -g
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
+ \frac{T}{m}
\begin{bmatrix}
\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi \\
\sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi \\
\cos\theta \cos\phi
\end{bmatrix}
```
In the body frame, the angular acceleration of the inertia $I \dot{\nu}$ , the centripetal forces $\nu \times (I\nu)$ and the gyroscopic forces $\Gamma$ are equal to the external torque $\tau$
```math
I\dot{\boldsymbol{\nu}} + \boldsymbol{\nu} \times (I\boldsymbol{\nu}) + \boldsymbol{\Gamma} = \boldsymbol{\tau}
```
```math
\dot{\boldsymbol{\nu}} = \mathbf{I}^{-1} \left(
  -\begin{bmatrix}
    \dot{\phi} \\ \dot{\theta} \\ \dot{\psi}
  \end{bmatrix}
  \times
  \begin{bmatrix}
    I_{xx} \dot{\phi} \\
    I_{yy} \dot{\theta} \\
    I_{zz} \dot{\psi}
  \end{bmatrix}
  - I_r
  \begin{bmatrix}
    \dot{\phi} \\ \dot{\theta} \\ \dot{\psi}
  \end{bmatrix}
  \times
  \begin{bmatrix}
    0 \\ 0 \\ 1
  \end{bmatrix}
  \omega_\Gamma
  + \boldsymbol{\tau}
\right)
```
```math
\begin{bmatrix}
\ddot{\phi} \\
\ddot{\theta} \\
\ddot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
\frac{(I_{yy} - I_{zz})\dot{\theta} \dot{\psi}}{I_{xx}} \\
\frac{(I_{zz} - I_{xx})\dot{\phi} \dot{\psi}}{I_{yy}} \\
\frac{(I_{xx} - I_{yy})\dot{\phi} \dot{\theta}}{I_{zz}}
\end{bmatrix}
- I_r
\begin{bmatrix}
\frac{\dot{\theta}}{I_{xx}} \\
-\frac{\dot{\phi}}{I_{yy}} \\
0
\end{bmatrix}
\omega_\Gamma
+
\begin{bmatrix}
\frac{\tau_\phi}{I_{xx}} \\
\frac{\tau_\theta}{I_{yy}} \\
\frac{\tau_\psi}{I_{zz}}
\end{bmatrix}
```
in which $\omega_\Gamma = \omega_1 - \omega_2 + \omega_3 - \omega_4$
```math
\boldsymbol{\tau}_B =
\begin{bmatrix}
\tau_\phi \\
\tau_\theta \\
\tau_\psi
\end{bmatrix}
=
\begin{bmatrix}
\frac{l}{\sqrt{2}} \left( T_1 + T_2 - T_3 - T_4 \right) \\
\frac{l}{\sqrt{2}} \left( -T_1 + T_2 + T_3 - T_4 \right) \\
\tau_{M_1} - \tau_{M_2} + \tau_{M_3} - \tau_{M_4}
\end{bmatrix}
```
```math
T = \sum_{i=1}^{4} f_i = k \sum_{i=1}^{4} \omega_i^2
```
```math
\mathbf{T}_B =
\begin{bmatrix}
0 \\
0 \\
T
\end{bmatrix}
```
```math
\tau_{M_i} = b \, \omega_i^2 + I_M \, \dot{\omega}_i
```
The lift constant is $k$, the drag constant is $b$ and the inertia moment of the rotor is $I_M$. Usually the effect of $\dot{\omega}_i$ is considered small and thus it is omitted.  $l$ is the distance between the rotor and the center of mass of the quadcopter. The quadcopter’s actuators limit the applicable thrust for each rotor, effectively constraining $T_i$, $i$={1,2,3,4} as
```math
0 \leq T_{\text{min}} \leq T_i \leq T_{\text{max}}
```
### Aerodynamical effects
The preceding model is a simplification of complex dynamic interactions. To enforce more realistical behaviour of the quadcopter, drag force generated by the air resistance is included. 
```math
\begin{equation}
\begin{bmatrix}
\ddot{x} \\
\ddot{y} \\
\ddot{z}
\end{bmatrix}
=
- g
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
+
\frac{T}{m}
\begin{bmatrix}
\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi \\
\sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi \\
\cos\theta \cos\phi
\end{bmatrix}
-
\frac{1}{m}
\begin{bmatrix}
A_x & 0 & 0 \\
0 & A_y & 0 \\
0 & 0 & A_z
\end{bmatrix}
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{z}
\end{bmatrix}
\end{equation}
```
where  $A_x$, $A_y$, and $A_z$ are the drag force coefficients for velocities in the corresponding directions of the inertial frame.
### State Equations
The nonlinear state-space dynamics combine translational and rotational dynamics:
```math
\dot{\mathrm{x}} = f(\mathrm{x},u)
```
with 
```math
\mathrm{x} = 
\begin{bmatrix}
x & y & z & \dot{x} & \dot{y} & \dot{z} & \phi & \theta & \psi & \dot{\phi} & \dot{\theta} & \dot{\psi}
\end{bmatrix}^T ; 

\mathbf{u} = 
\begin{bmatrix}
T_1 & T_2 & T_3 & T_4
\end{bmatrix}^T
```
### Continous Optimal Control Formulation: Trajectory Optimisation
```math
\begin{align*}
\min_{\mathbf{u}(\cdot)} J = \int_{0}^{T} \ell(\mathbf{x}(\tau), \mathbf{u}(\tau))\mathrm{d}\tau + V_f^e(\mathbf{x}(T))\\
\text{subject to} \quad  \forall \tau \in [0, T]:\\
& \dot{\mathbf{x}}(\tau) = f(\mathbf{x}(\tau), \mathbf{u}(\tau)), \quad \mathbf{x}(0) = \mathbf{x}_0 \\
& \mathbf{x}(\tau) \in \mathbb{X}, \quad \mathbf{u}(\tau) \in \mathbb{U}
\end{align*}
```
### Discrete time Nonlinear Model Predictive Control Formulation: Trajectory Optimisation
The control problem is to regulate the state to the origin. In discrete time NMPC formulation, this is done by considering the optimization problem at current state $x_k$ defined by:  
```math
\begin{align*}
V(\mathrm{x}_k) = \min_{\mathbf{u}_k, \mathbf{x}_k} \; & V_f(\mathrm{x}_{k+N|k}) + \sum_{i=0}^{N-1} L(\mathrm{x}_{k+i|k}, \mathrm{u}_{k+i|k}) \\
\text{s.t.} \quad & \mathrm{x}_{k|k} = \mathrm{x}_k, \\
& \mathrm{x}_{k+i+1|k} = f_{rk4}(\mathrm{x}_{k+i|k}, \mathrm{u}_{k+i|k}), \quad \forall i \in \mathbb{I}_{[0, N-1]}, \\
& \mathrm{x}_{k+i|k} \in \mathbb{X}, \quad \mathrm{u}_{k+i|k} \in \mathbb{U}, \\
& \mathrm{x}_{k+N|k} \in \mathbb{X}_f.
\end{align*}
```
```math
L( \mathrm{x}_{k+i|k},  \mathrm{u}_{k+i|k}) = \left\| \mathrm{x}_{k+i|k} -  \mathrm{x}_r \right\|_Q^2 + \left\|  \mathrm{u}_{k+i|k} -  \mathrm{u}_r \right\|_R^2;      V_f( \mathrm{x}_{k+N|k}) = \left\|  \mathrm{x}_{k+N|k} -  \mathrm{x}_r \right\|_P^2
```
Q,R,P positive semi definite weight matrices:
