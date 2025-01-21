# WeCars 
Wecars is an add on package to the Webots Robotics Simulation Software. It enables higher fidelity vehicle dynamics simulation than offered in Webots by default.
The goal of this package is to increase the accesiblity of vehicle dynamics simulation, as well as vehicle dynamics in general more accessible. 
Almost all modern software packages are either prohibitively expensive, unmaintained, or otherwise inacessible. In addition, information on the 
steady state and dynamic stabilty of automotive vehicles are not widely understood yet frequently misquoted and misunderstood. This software package 
intends to allow individuals to gain understanding through visualization and simulation in tandem the fundamental physics.
## Features: 
1) Real time simulation of Double Wishbone and Mcpherson Strut suspension behavior. (Trailing Arm, Semi Trailing Arm, and Multilink are planned) 

        A) Generation of Camber and Toe curves under dynamic conditions.
        B) Dynamic generation of platform stability graphs (Planned/Stretch)
        C) Roll and Pitch center adjustment scripts (Planned)

2) Dynamic Tyre Model simulation support. Linear, nonlinear, and 3D tabular tires are supported. (May include support for other models in the future)
3) Procedural Vehicle Generation from JSON Files (Planned)
4) Single point preview real time vehicle simulation (Planned)
5) Generation of G-G diagrams, Milliken Moment Method diagrams, etc (Planned)
6) Racetrack Laptime Simulation (Planned)
7) Racetrack and road generation from CSV Files (Planned)
8) Advanced Drivetrain Simulation (Stretch Goal) 
9) Dynamic Aero Map Support (Stretch Goal)
10) Tire data reconstruction (Stretch Goal)
    
    


Credits: 
The Geometry Generation Code in the Single Suspension Proto was done by Professor Alexander Brown at Lafayette College. https://github.com/Alexanderallenbrown 
Joshua A. Marshall, How to Implement a First-Order Low-Pass Filter in Discrete Time, 2021, URL: https://github.com/botprof/first-order-low-pass-filter.
