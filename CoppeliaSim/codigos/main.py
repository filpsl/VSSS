import time
import math
import sims.sim as sim
import signal
import sys

def connect_CRB(port):
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

class Corobeu:
    def __init__(self, kp, ki, kd, dt):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.f_ant = 0
        self.interror = 0
        self.Integral_part = 0
        
        self.v_max = 8
        self.v_min = -8
        self.v_linear = 8
        self.phi = 0
        
        self.last_speed_time = time.time()
        
        if COR_DO_TIME == 1:
            self._robot_attr = "robots_blue"
        elif COR_DO_TIME == 0:
            self._robot_attr = "robots_yellow"
        else: 
            raise ValueError(f"COR_DO_TIME: {COR_DO_TIME} é inválido, altere-o no 'config_ideal.py'.")
        
        signal.signal(signal.SIGINT, self.off)
        signal.signal(signal.SIGTERM, self.off)
    

    def speed_control(self, U, omega):

        vr = (2 * U + omega * 7.5) / 3
        vl = (2 * U - omega * 7.5) / 3
        
        # Encontra o fator de escala necessário
        max_speed = max(abs(vr), abs(vl))
        if max_speed > self.v_max:
            scale_factor = self.v_max / max_speed
            vr *= scale_factor
            vl *= scale_factor
        
        if math.isnan(vr) or math.isnan(vl):
            vr, vl = 0, 0
        
        return int(vl), int(vr)

    
    def follow_ball(self):        
        phi_obs = 0
        
        (clientID, robot, motorE, motorD, ball) = connect_CRB(19995)
        
        if (sim.simxGetConnectionId(clientID) != -1):
            
            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
            
            while True:
                                
                s, robotPosition = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                s, ballPosition = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                s, phi = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)

                x = robotPosition[0]
                y = robotPosition[1]
                ball_x = ballPosition[0]
                ball_y = ballPosition[1]
                phi_obs = phi[2] - math.pi/2
                
                if x is None or y is None:
                    continue

                phid = math.atan2((ball_y - y), (ball_x - x))
                phid = self.wrap_angle(phid)
                phi_obs = self.wrap_angle(phi_obs)

                error_phi = self.wrap_angle(phid - phi_obs)                
                omega = self.pid_controller(error_phi)

                error_distance = math.sqrt((ball_y - y)**2 + (ball_x - x)**2)
                error_distance_global = math.sqrt((ball_y - y) ** 2 + (ball_x - x) ** 2)

                U = self.v_linear
                current_time = time.time()
                if current_time - self.last_speed_time >= self.dt:

                    vl, vr = self.speed_control(U, omega)
                    
                    sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, vr, sim.simx_opmode_blocking)

                    self.last_speed_time = current_time


    def follow_path(self, path_x, path_y):
        phi_obs = 0

        while True:
            x, y, phi_obs = self.get_position()[0:3]

            if x is None or y is None:
                continue

            phid = math.atan2((path_y - y), (path_x - x))
            phid = self.wrap_angle(phid)
            phi_obs = self.wrap_angle(phi_obs)
            
            error_phi = self.wrap_angle(phid - phi_obs)
            omega = self.pid_controller(error_phi)

            error_distance = math.sqrt((path_y - y)**2 + (path_x - x)**2)
            error_distance_global = math.sqrt((path_y - y) ** 2 + (path_x - x) ** 2)
            
            U = self.v_linear  
            current_time = time.time()

            if current_time - self.last_speed_time >= self.dt:
                
                vl, vr = self.speed_control(U, omega)
                self.send_speed(vl, vr)
                
                self.last_speed_time = current_time
            
            if (error_distance <= 0.07):
                self.off()
            

    def pid_controller(self, error):
        
        Integral_saturation = 5
        raizes = math.sqrt(kd), math.sqrt(kp), math.sqrt(ki)
        Filter_e = 1 / (max(raizes) * 10)   
        unomenosalfaana = math.exp(-(self.dt / Filter_e))
        alfaana = 1 - unomenosalfaana
        self.interror += error
        f = unomenosalfaana * self.f_ant + alfaana * error
        self.f_ant = f
        deerror = (f - self.f_ant) / self.dt if self.f_ant != 0 else f / self.dt
        self.Integral_part = min(max(self.Integral_part + ki * self.interror * self.dt, -Integral_saturation), Integral_saturation)
        PID = kp * error + self.Integral_part + deerror * kd
        return PID
        
        return omega
    
    
    def wrap_angle(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi
    
    
    def off(self, signum=None, frame=None ):
        sys.exit()
        return
    

if __name__ == "__main__":
    ROBOT_ID = 1
    ROBOT_PORT = 80
    COR_DO_TIME = 0
    
    # kp = 0.3629
    # ki = 0.1891
    # # Ki = 0    
    # kd = 0.1
    
    # kp = 3.195983       #
    # ki = 0.02434838     # ERRO: 46
    # kd = 0.27948516     #
    
    # kp = 3.09724156         #
    # ki = 0.02232751         # ERRO: 44
    # kd = 0.22432429         #
    
    kp = 3.0528502  
    kd = 0.79546531
    ki = 0
    
    dt = 0.05
    
    # Kp = 10
    # Ki = 3.63
    # Kd = 2.46
    
    crb01 = Corobeu(kp, ki, kd, dt)
    crb01.follow_ball()