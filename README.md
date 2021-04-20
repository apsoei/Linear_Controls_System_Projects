# Linear Controls System Projects

## By Jae seok Oh

### Description
Control system simulations using webot.

## Requirements.

- Webot simulator https://www.cyberbotics.com/ version R2020b - rev 1.

- Native Python (i.e anaconda version does not work with webot)

- Numpy, Scipy, Matplotlib

## To run.

To run one of the projects.

Open webot.

Click open World within one of the project folders ( e.g) P1/code/worlds/ ).

Click run to simulate controllers.

For P5, comment out one of the following lines to test LQR or MRAC in AdaptiveController.py.

`# customController = AdaptiveController(driver, lossOfThust)`
`customController = LQRController(driver, lossOfThust)`


### Tesla model 3 simulation from EKF SLAM.

https://user-images.githubusercontent.com/50928257/115320727-64628080-a1bd-11eb-9e11-3da5aec181ce.mov
 

## Project 1 (P1)

Vechicle (Tesla Model 3) PID control simulation

Trajectory plot

![P1_Plot](https://user-images.githubusercontent.com/50928257/115320880-a390d180-a1bd-11eb-968b-43584c8ffce9.png)


## Project 2 (P2)

Vechicle (Tesla Model 3) full-state feedback control simulation



## Project 3 (p3)

Vechicle (Tesla Model 3) Discrete time Infinite Horizon Linear Quadratic Regulator (LQR) control simulation

## Project 4 (p4)

Vechicle (Tesla Model 3) EKF SLAM simulation

## Project 5 (P5)

Quadrotor Drone (DJI Mavic Pro 2) LQR and Model Reference Adaptive Control (MRAC) simulation

Scenario: Drone loses thrust by 68% in one of the motors around 15 seconds. Comparison between LQR and MRAC.

LQR simulation (loss thrust: 68%)

https://user-images.githubusercontent.com/50928257/115318659-22374000-a1b9-11eb-86df-06bed9cc34a1.mov

MRAC simulation (loss thrust: 68%)

https://user-images.githubusercontent.com/50928257/115318805-78a47e80-a1b9-11eb-9de1-762a5868f3ad.mov

Plots

![LQR_MRAC_plot](https://user-images.githubusercontent.com/50928257/115318827-84904080-a1b9-11eb-8efb-046cba70a336.png)



