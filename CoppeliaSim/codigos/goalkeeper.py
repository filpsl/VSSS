""""
follow_ball.py

This class present the PID controller implementation on the mobile robot follower ball, this class present five functions
the first function called init, make the initialization of the variables, the connect_CRB present the connection with
the coppeliaSim, Speed_CRB calculated the wheels speeds based on the robot topology (in this case is a differential robot),
PID_Controller_phi implement the PID based on the phi error (see video of the georgia tech course in this link:
https://www.youtube.com/watch?v=Lgy92yXiyqQ&list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr&index=14) finally, Robot_CRB is the 
principal function in this class.


Author:

    Mario Andres  Pastrana Triana (mario.pastrana@ieee.org)
    Matheus de Sousa Luiz (luiz.matheus@aluno.unb.br)
    EVA/MARIA PROJECT - University of Brasilia-(FGA)
    

Release Date:
    JUN 19, 2024

Finally comment:
    Querido lector por ahora, la correcta implementación de este código lo sabe Mario, Dios, la Virgen Maria y los santos
    esperemos que futuramente Mario continue sabiendo como ejecutar el código o inclusive que más personas se unan para
    que el conocimiento aqui depositado se pueda expandir de la mejor manera.
    Let's go started =)
"""""


import math
import sim
import numpy as np
import math as mat
import time

class Corobeu():
    """"
    Corobeu class is the principal class to controller the EVA positions robots.
        __init___               = Function to inicializate the global variables
        connect_CRB             = Present the connection with the coppeliaSim
        Speed_CRB               = Calculated the wheels speeds based on the robot topology (in this case is a differential robot)
        PID_Controller_phi      = Implement the PID based on the phi error
        Robot_CRB               = The principal function in this class.
    """""

    def __init__(self):

        """"
            Initializations global variables
            self.y_out    (float list) = Out position robot in the y axis
            self.x_out    (Float list) = Out position robot in the x axis
            self.phi      (Float)   = phi robot value
            self.v_max    (Integer) = max speed for the robot
            self.v_min    (Integer) = min speed for the robot
            self.posError (Float list) = phi error
        """""

        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max = 8
        self.v_min = -8
        self.v_linear = 5
        self.posError = []

    def connect_CRB(self, port):
        """""
        Function used to communicate with CoppeliaSim
            argument :
                Port (Integer) = used to CoppeliaSim (same CoppeliaSim)
                
            outputs : 
                clientID (Integer)  = Client number
                robot    (Integer)  = objecto robot
                MotorE   (Integer)  = Object motor left
                MotorD   (Integer)  = Object motor right
                ball     (Integer)  = Object ball on the scene
        """""

        ### Connect to coppeliaSim ###

        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Connect to", port)
        else:
            print("Can not connect to", port)

        ### Return the objects ###

        returnCode, robot = sim.simxGetObjectHandle(clientID, 'robot01', 
                                                    sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'motorL01',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'motorR01',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking)
               
        return clientID, robot, MotorE, MotorD, ball
    
    def Speed_CRB(self, U, omega, error_distance, ballY, robotY):

        """""
        Function used to calculate the speed for each wheel based on the topology robot
            argument :
                U              (Integer) = Max linear speed
                omega          (Float)   = Angular speed
                error_distance (Float)   = Error between the robot and the final point

            outputs : 
                vl (Float)   = Left speed
                vd (Float)   = right speed
                a  (Integer) = condition to simulate
        """""

        ### Calculate the speed based on the topology robot ###

        vd = (2 * U + omega * 7.5) / (2 * 3.2)
        vl = (2 * U - omega * 7.5) / (2 * 3.2)
        a = 1
        print(f'vl === {vl} vd == {vd}')
        print(f' omega == {omega}')
        ### the first time #####

        if omega >= 1 or omega <= -1:
            Max_Speed = 0.01
            Min_Speed = -0.01
        else:
            Max_Speed = self.v_max
            Min_Speed = self.v_min

        # Max_Speed = self.v_max
        # Min_Speed = self.v_min

        ### Saturation speed upper ###

        if vd >= Max_Speed:
            vd = Max_Speed
        if vd <= Min_Speed:
            vd = Min_Speed

        ### Saturation speed Lower ###

        if vl >= Max_Speed:
            vl = Max_Speed
        if vl <= Min_Speed:
            vl = Min_Speed

        if robotY > ballY:
            vl = -vl
            vd = -vd

        ### When arrive to the goal ###

        if error_distance <= 0.03:
            # a = 0
            vl = 0
            vd = 0

        ### Return values ###

        return vl, vd, a
    
    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):

        """""
        Function used to calculate the omega value (output PID) used on the speed function
            argument :
                kp              (Float) = Proportional constant used on the PID controller
                ki              (Float) = Integral constant used on the PID controller
                kd              (Float) = Derivative constant used on the PID controller
                deltaT          (Float) = Sample time
                error           (Float) = Phi error between the robot and the goal point
                interror        (Float) = Error Integral
                fant            (Float  = Used on the derivative part
                Integral_part   (Float) = Integral part

            outputs : 
               PID              (Float) = Omega value used on the speed function
               f                (Float) = f value use on the derivative part
               interror         (Float) = Error Integral
               Integral_part    (Float) = Integral part
        """""

        ### Max value of saturation in the integral part ###

        Integral_saturation = 10

        ### Find the filter e ####

        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        # print(f'mayor == {mayor}')
        Filter_e = 1 / (mayor * 10)

        ### Calculate the derivative part ###

        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f/deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        ### Calculate the integral part ###

        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        ### Calculate the omega value (PID output) ###

        PID = kp * error + Integral_part + deerror * kd

        ### Return the principal values ###
        # print(f'Integral_part == {Integral_part}')
        return PID, f, interror, Integral_part

    def Robot_CRB(self, kpi, kii, kdi, deltaT):

        """""
        Principal function to simulate the follower ball robot
            argument :
                kpi              (Float) = Proportional constant used on the PID controller
                kii              (Float) = Integral constant used on the PID controller
                kdi              (Float) = Derivative constant used on the PID controller
                deltaT          (Float) = Sample time

            outputs : 
               none
        """""

        ### Get the objects within coppeliaSim using the connect_CRB function ###

        (clientID, robot, motorE, motorD, ball) = self.connect_CRB(19995)

        ### Criterio to simulation ###

        a = 1

        ### Init the principal values ###

        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        cont0 = 0

        ### Make the communication with coppeliaSim ###

        if (sim.simxGetConnectionId(clientID) != -1):

            ### Criterio to simulation ###

            while (a == 1):

                ### important to get valid values and init the phi value ###

                if Number_Iterations <= 1:
                    
                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                    s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # self.phi = angle_robot[2] - 1.47
                    self.phi = 0
                    sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
                else:

                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                    # s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # s, angle_ball = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # self.phi = angle_robot[2] - 1.47

                    # print(f'Angle robot ==> {angle_robot}')

                    ### Calculate the phid (see georgia tech course) ###
                    
                    phid = math.atan2(ballPos[1] - positiona[1], -0.7 - positiona[0])
                    ### Phi error to send the PID controller

                    error_phi = phid - self.phi
                    # error_phi_degree = math.degrees(error_phi)
                    # print(f'error degree == > {error_phi_degree}, error ==> {error_phi}' )
                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
                                                                                               error_phi, interror_phi,
                                                                                               fant_phi,
                                                                                               Integral_part_phi)

                    ### Calculate the distance error, the robot stop when arrive the ball ###

                    error_distance = math.sqrt((ballPos[1] - positiona[1]) ** 2)

                    ### Acumulative distance error ###

                    self.posError.append(error_distance)

                    ### Implement the PID controller ###

                    ### Update the phi robot value ###

                    self.phi = self.phi + omega * deltaT

                    ### Calculate the speed right and left based on the topology robot ###

                    vl, vd, a = self.Speed_CRB(self.v_linear, omega, error_distance, ballPos[1], positiona[1])


                    ### Send the speed values to coppeliasim simulato ###

                    sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)

                    ### update the time simulation and the simulation iteration

                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)
                # time.sleep(0.5)
        ### Save the robot position ###

        self.y_out.append(positiona[1])
        self.x_out.append(positiona[0])

if __name__ == "__main__":

    crb01 = Corobeu()
    kpi = 20
    kii = 1
    kdi = 0.001
    deltaT = 0.05 # 50 ms
    crb01.Robot_CRB(kpi, kii, kdi, deltaT)