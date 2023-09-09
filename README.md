# state_observer_four_wheel_model
Linear state observer that estimates lateral velocity using measured signals from a four-wheel model

# RUNNING THE MODEL
a_entry_point.m                 -> As the name suggests, this is the entry script. Simply hitting PLAY inside this scrpt wil result running the observer simulation and plots the results
input_script.m                  -> To edit inputs
state_observer.m                -> Wrapper function which is called by the ODE function. This function contains observer algorithms. This function calls the core vehicle model from within
vehicle_model_fw_simplified.m   -> Core function containing the equations of motion of the vehicle model. This is function has only 1 role, accept some inputs and states, and calculate the forces and accelerations

# DESCRIPTION
This repository contains a linear state observer. Its goal is to use an estimator model to estimate a measured signal. 
In this repository, we don't use real measured signals. Instead a four-wheel model (with considerably more complexity) is assumed to be the real vehicle giving output signals. 
The yaw-rate from this vehicle model is considered to be measured output which the estimator is trying to track. 

The estimator is a simplified linear bicycle model, the yaw rate is tracked and the lateral velocity is the state to be estimated. 
Pole placement technique is used to calculate the observer gains
