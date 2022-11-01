## Author Hakim Amer
## Date: 28/10/2022


import setup_path
import airsim
import cv2
import threading
import time
import numpy as np
import quaternion
import math
import acado
import sys, os
import argparse

sys.path.append(os.path.join(os.path.dirname(sys.path[0]),'quadrotor_mpc_codegen'))  #path to c-code 

# solver for optimal control.


class ControlInput(object):    #define control input 
    def __init__(self, throttle, roll_rate, pitch_rate, yaw_rate):
        self.throttle = throttle
        self.yaw_rate = -yaw_rate
        self.pitch_rate = -pitch_rate
        self.roll_rate = roll_rate

class MPC(object):
    def __init__(self, x_ref = 0.0, y_ref = 0.0, z_ref = 2.0 , horizon = 1):
        # Iteration for approximating mpc. 
        self.max_iteration = 15
        # Cost of mpc.  
        # TODO: make weights parameters read from a config file 
        self.Q = np.diag([50.0, 50.0, 100.0, 100.0, 100.0, 100.0, 100.0, 20.0, 20.0, 30.0, 1.0, 1.0, 1.0, 1.0])
        self.Qf = self.Q[:10, :10]
        # Step size and horizon
        self.horizon = 20
        # Copy from acado.
        # TODO: make number of variables read from acado c code
        self.NX = 10
        self.NY = 14
        self.NYN = 10
        self.NU = 4

        self.reference = np.array( [2.0, 2.0, -2.0,  1.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #[x,y,z,u,v,w,qw,qx,qy,qz,p,q,r,T]


        self.THRESHOLD = 0.01
        self.use_ith_control = 1
        # Internal checks.
        self.max_deviation_dist = 2.0
        self.max_deviation_angle = 12.0
        self.max_sudden_angle = 1.2
        self.acc_deviation_angle = 0
        self.discount = 0.25
        self.g = 9.81
        self.last_checked = None
        self.saturated = False

        #airsim API          
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

   # set the reference of the MPC 
    def setReference(self, current):
        assert(self.NY == self.reference.shape[0])
        yn = np.zeros((1, self.NYN))
        Y = np.zeros((self.horizon, self.NY))   
        yn[0,:] = self.reference[:1]
        Y[:,:] = self.reference[:self.horizon]
         
        return (Y, yn)

   ##OCP stuff 
    ## create state feeback vector for acado
    def toAcadoState(self, state):
        x = np.zeros((1, self.NX))
        x[0,0] = state.position.x_val
        x[0,1] = state.position.y_val
        x[0,2] = state.position.z_val
        x[0,3] = state.orientation.w_val
        x[0,4] = state.orientation.x_val
        x[0,5] = state.orientation.y_val
        x[0,6] = state.orientation.z_val
        x[0,7] = state.linear_velocity.x_val
        x[0,8] = state.linear_velocity.y_val
        x[0,9] = state.linear_velocity.z_val
        print("x", x[0,0])
        print("y", x[0,1])
        print("z", x[0,2])
        return x

  #Get input and prediction along prediction horizon 
    def getFullMpcOutput(self, state):
        # finish iter
        X = np.zeros((self.horizon + 1, self.NX))
        U = np.zeros((self.horizon, self.NU))
        prev_x = np.zeros((1, self.NX))
        ref_traj, terminal_state = self.setReference(state)
        for i in range(self.max_iteration):
            X, U = acado.mpc(0, 1, self.toAcadoState(state), X, U, ref_traj, terminal_state, np.transpose(np.tile(self.Q,self.horizon)), self.Qf, 0)
            #X, U = acado.mpc(0, 1, self.toAcadoState(state), X, U, ref_traj, terminal_state, self.Q, self.Qf, 0)
            if (np.linalg.norm(X-prev_x) < self.THRESHOLD):
                # print("CONTROL: Input mpc terminating iteration at ", i)
                break
            prev_x = X #Update prev
        return (X,U)   

  #Get the first input, which is to be applied by the MPC
    def getInput(self, state):
        X, U = self.getFullMpcOutput(state)
        T_0 = U[0,0]/2/self.g
        u_body_0 =  U[0,1:]
        u_body =  U[self.use_ith_control,1:]
        T = U[self.use_ith_control,0]/2/self.g
        return ControlInput(T,u_body[0],u_body[1],u_body[2])


   ## send control commands to AirSim
    def apply_control(self):
        
        drone_state = self.client.getMultirotorState().kinematics_estimated   #get drone state
        input = self.getInput(drone_state)                        #apply control input
        print("Thrust ", input.throttle)
        print("roll rate ", input.roll_rate * 57.3)
        print("pitch rate ", input.pitch_rate * 57.3)
        print("yaw rate ", input.yaw_rate * 57.3 )
        self.client.moveByAngleRatesThrottleAsync(input.roll_rate, input.pitch_rate, input.yaw_rate, input.throttle, duration=0.05).join()


if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("mpc_control.py makes mpc controller for waypoint following: Please specify waypoints")
    arg_parser.add_argument("--x_ref", type=float, help="reference x coordinate", default=2.0)
    arg_parser.add_argument("--y_ref", type=float, help="reference x coordinate", default=2.0)
    arg_parser.add_argument("--z_ref", type=float, help="reference x coordinate", default=2.0)
    args = arg_parser.parse_args(args)      
    mpc_control=MPC(args.x_ref, args.y_ref, args.z_ref , 20)
    true=1
    while true:  
          mpc_control.apply_control()
          


  


