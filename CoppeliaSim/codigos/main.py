import time
import math
import sim
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
    def __init__(self, kp, ki, kd, dt, omega_max):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        # --- Limites de Saída (para anti-windup e segurança) ---
        self.omega_min = -omega_max
        self.omega_max = omega_max
        
        # --- Filtro para o Termo Derivativo (para suavizar ruídos) ---
        # Um valor de 'alpha' próximo de 1 significa pouca filtragem.
        # Um valor próximo de 0 significa muita filtragem.
        # Uma boa regra é começar com um alpha ~0.5 e ajustar.
        self.filter_alpha = 0.5 
        
        # --- Variáveis de Estado (precisam ser lembradas entre as chamadas) ---
        self.integral = 0.0
        self.previous_error = 0.0
        self.filtered_previous_error = 0.0
        
        self.v_max = 8
        self.v_min = -8
        self.v_linear = 4
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

                    # print(f"PHI_OBS= {phi_obs}\\ERRO: {error_phi}\\-------------------\\")
                    vl, vr = self.speed_control(U, omega)
                    print(vl,vr)
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
        # --- 1. Termo Proporcional (P) ---
        # A resposta instantânea ao erro atual.
        proportional_term = self.kp * error
        
        # --- 2. Termo Derivativo (D) com Filtro ---
        # Previne o "derivative kick" (pico na derivada) e suaviza ruídos.
        # Primeiro, filtramos o erro atual.
        filtered_error = (self.filter_alpha * error) + (1 - self.filter_alpha) * self.filtered_previous_error
        
        # Calculamos a derivada sobre o erro filtrado.
        derivative_term = self.kd * (filtered_error - self.filtered_previous_error) / self.dt
        
        # --- 3. Termo Integral (I) com Anti-Windup Condicional ---
        # Acumula o erro para eliminar o erro em regime estacionário.
        # A lógica de anti-windup é aplicada aqui.
        potential_integral = self.integral + error * self.dt
        
        # Só atualizamos a integral se a saída PROVISÓRIA estiver dentro dos limites.
        # Isso evita que a integral cresça indefinidamente quando a saída já está no máximo.
        provisional_omega = proportional_term + self.ki * potential_integral + derivative_term
        if self.omega_min < provisional_omega < self.omega_max:
            self.integral = potential_integral
            
        integral_term = self.ki * self.integral
        
        # --- 4. Soma Final e Saturação da Saída ---
        # Combina os três termos para obter a saída final do controlador.
        omega = proportional_term + integral_term + derivative_term
        
        # Garante que a saída final NUNCA exceda os limites definidos.
        omega = max(min(omega, self.omega_max), self.omega_min)
        
        # --- 5. Atualização das Variáveis de Estado para a Próxima Iteração ---
        self.previous_error = error
        self.filtered_previous_error = filtered_error
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
    
    Kp = 0.3629
    # Ki = 0.1891
    Ki = 0    
    Kd = 0.0001
    
    dt = 0.1
    omega_max = 8
    
    # Kp = 10
    # Ki = 3.63
    # Kd = 2.46
    
    crb01 = Corobeu(Kp, Ki, Kd, dt, omega_max)
    crb01.follow_ball()