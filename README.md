# Nonlinear MPC for Regenerative Braking of Metropolia Motorsports Electric Car

This repository contains a "Nonlinear Model Predictive Control (NMPC)" setup for regenerative braking in Metropolia Motorsports' electric race car. 
The goal is to manage both "vehicle speed (v)" and "State of Charge (SoC)" while applying regenerative braking torque.

## Table of Contents
1. [Introduction](#introduction)
2. [Project Structure](#project-structure)
3. [Model Details](#model-details)
   1. [States and Inputs](#states-and-inputs)
   2. [System Equations](#system-equations)
   3. [Constraints](#constraints)
   4. [Weights and Tuning](#weights-and-tuning)
4. [Running the Project](#running-the-project)
5. [Simulink Model Explanation](#simulink-model-explanation)
6. [Reference Signals](#reference-signals)
7. [How It All Fits Together](#how-it-all-fits-together)

## Introduction

Regenerative braking allows energy typically lost as heat during braking to be captured and stored in the electric car’s battery, improving range and overall efficiency. 
In this project, we use "Nonlinear Model Predictive Control (NMPC)" to:
- Track the desired vehicle speed (velocity).
- Maintain the battery’s State of Charge (SoC) within acceptable limits.
- Calculate and apply an optimal torque (positive or negative) that balances performance and energy recovery.

The NMPC algorithm considers constraints on both the "speed" and "SoC", as well as bounds on the "regenerative torque".


## Project Structure

1. "regen_script.m`" (Matlab Script):
   - Sets up the NMPC controller via a function (`myNLMPCSetup`) that defines:
     - State dimensions and sample time.
     - Prediction and control horizons.
     - Constraints on outputs (velocity and SoC).
     - Constraints on the manipulated variable (regen torque).
     - Cost function weights.
   - Creates an `nlobj` object in the Matlab workspace to be used within Simulink or other scripts.

2. "Simulink Regen Model" (`regen_braking_model.slx`):
   - Implements the NMPC block and ties in the plant model and constraints.
   - Allows you to run closed-loop simulations of the electric car’s speed and SoC under various driving conditions.

3. "Reference Signals":
   - There is a separate file (ref.mat) created with the "Signal Builder":
     1. "Velocity reference signal" (`v_ref`).
     2. "SoC reference signal" (`SoC_ref`).
   - These signals are fed into the NMPC controller so it knows the desired speed and SoC trajectory over time.

4. "Plant Model" :
   - A custom Simulink block or subsystem that contains the vehicle dynamics.
   - Uses the equations of motion to simulate the real-world behavior of the vehicle under different torques, resistances, and battery states.

5. "Function Inside the Plant Model" (the last file mentioned):
   - A supporting function (or set of functions) that define the exact state equations:
     - "`myStateFcn`" calculates the time derivatives of the states.
     - "`myOutputFcn`" returns the measured outputs (in this case, velocity and SoC).


## Model Details

### 1. States and Inputs

 "States": 
  1. \( x_1 = v \) (vehicle speed, m/s)
  2. \( x_2 = \{SoC} \) (battery State of Charge, unitless from 0 to 1)

- "Input":
  - \( u = \{regen} \) (regenerative torque command; negative indicates braking torque, positive can indicate driving torque)

  "Outputs":
  1. \( y_1 = v \)
  2. \( y_2 = \{SoC} \)

### 2. System Equations

From the local functions `myStateFcn` and `myOutputFcn`:

1. "Dynamics" (`myStateFcn`):
 
Where:
- m       : Vehicle mass
- r       : Wheel radius
- rho     : Air density
- Cd      : Aerodynamic drag coefficient
- A       : Frontal area
- g       : Gravitational acceleration
- Crr     : Rolling resistance coefficient
- eta_chg : Regeneration efficiency
- Enom    : Nominal battery energy scaling

The output function (`myOutputFcn`) is simply:
y = [ v SoC ]


### 3. Constraints

"Output Constraints":  
  - \( v >= 0 \) (no negative speed in this scenario)  
  - \( 0.2 <= SoC <= 1.0 \) (maintain battery between 20% and 100%)

  "Input (Regen Torque) Constraints":  
  - \(\{regen} \in [-100, 100]\)

### 4. Weights and Tuning

 "Output Tracking Weights":
  - `[1  1]` for speed and SoC, respectively (equal priority).
  "Manipulated Variable (Regen) Weight":
  - `ManipulatedVariables = 0` (no penalty on absolute torque).
 "Rate of Change Weight":
  - `ManipulatedVariablesRate = 0.1` (avoid abrupt changes in torque).


## Running the Project

1. "Clone or Download" this repository.
2. Open MATLAB and add the repository folder to your MATLAB path.
3. Run "`regen_script.m`":
   - This creates an `nlobj` object in the base workspace with the NMPC settings.
4. Open the "Simulink model" (e.g., `regen_braking_model.slx`).
5. Load or reference the "`ref.mat`" file in the Signal Builder for the velocity and SoC references.
6. "Run" the Simulink simulation:
   - Observe how the NMPC tracks the desired speed and SoC.
   - Explore signals such as the torque command, actual speed, and SoC over time.



## Simulink Model Explanation

Inside the Simulink model, you will typically see:
- "NMPC Controller Block": Uses the `nlobj` from the MATLAB workspace.
- "Plant Subsystem": Implements the EV model (`myStateFcn`) using the dynamic equations.
- "Reference Signal": A single source that provides both velocity and SoC target trajectories.
- "Scope Blocks": Let you monitor how well the controller tracks the references and respects constraints.



## Reference Signals

A ref.mat` file contains:
- "Velocity reference"  
- "SoC reference"

These signals can be modified to test different driving scenarios (e.g., racing laps, city driving, etc.).


## How It All Fits Together

1. "`regen_script.m`" initializes the NMPC object with constraints, horizons, and weights.
2. "Simulink" calls on `nlobj` each timestep to compute the optimal regenerative (or motoring) torque.
3. The "Plant Model" simulates the electric vehicle dynamics (speed and SoC).
4. The NMPC controller compares the *current speed and SoC* with the *desired references* from `ref.mat` and updates the torque command to minimize errors while respecting constraints.



"Thank you for checking out the Nonlinear MPC for Regenerative Braking model!" 
If you have any questions or suggestions, feel free to open an issue or submit a pull request.
