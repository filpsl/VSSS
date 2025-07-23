import numpy as np
import socket
import time
import math
import struct
import signal
import configs.wrapper_pb2 as wr
import sys
from config import IP_ARES, ID_ARES, COR_DO_TIME

import numpy as np

class KalmanFilter:
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        """
        Inicializa o Filtro de Kalman.
        :param dt: Intervalo de tempo entre os passos do filtro.
        :param u_x, u_y: Aceleração (entrada de controle), geralmente 0 para objetos não controlados.
        :param std_acc: Desvio padrão da aceleração (ruído do processo).
        :param x_std_meas, y_std_meas: Desvio padrão do ruído da medição.
        """
        self.dt = dt
        self.u = np.array([[u_x], [u_y]])

        # Vetor de estado inicial [px, py, vx, vy]T
        self.x = np.zeros((4, 1))

        # Matriz de transição de estado F (4x4)
        self.F = np.array([[1, 0, self.dt, 0],
                           [0, 1, 0, self.dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        # Matriz de entrada de controle B (4x2)
        self.B = np.array([[(self.dt**2)/2, 0],
                           [0, (self.dt**2)/2],
                           [self.dt, 0],
                           [0, self.dt]])

        # Matriz de medição H (2x4)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        # Covariância do ruído do processo Q (4x4)
        # Modela a incerteza do modelo de movimento (ex: acelerações inesperadas)
        self.Q = np.array([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                           [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                           [(self.dt**3)/2, 0, self.dt**2, 0],
                           [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2

        # Covariância do ruído da medição R (2x2)
        # Modela a incerteza do sensor (visão)
        self.R = np.array([[x_std_meas**2, 0],
                           [0, y_std_meas**2]])

        # Matriz de covariância da estimativa P (4x4)
        # Representa a incerteza do nosso vetor de estado
        self.P = np.eye(self.F.shape[0])

    def predict(self):
        """
        Etapa de Predição do Filtro de Kalman.
        """
        # Prediz o estado
        self.x = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        
        # Prediz a covariância do erro
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        """
        Etapa de Atualização (Correção) do Filtro de Kalman.
        :param z: Medição atual (ex: [px, py] do sistema de visão) como um vetor coluna.
        """
        # Inovação ou resíduo da medição
        y = z - np.dot(self.H, self.x)
        # Covariância da inovação
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        # Ganho de Kalman ótimo
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # Atualiza a estimativa do estado
        self.x = self.x + np.dot(K, y)
        # Atualiza a covariância do erro
        I = np.eye(self.F.shape[0]) # Matriz identidade 4x4
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        

def init_vision_socket(VISION_IP="224.5.23.2", VISION_PORT=10015):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                    struct.pack("=4sl", socket.inet_aton(VISION_IP), socket.INADDR_ANY))
    sock.bind((VISION_IP, VISION_PORT))
    return sock

class Corobeu:
    def __init__(self, robot_ip, robot_port, robot_id, vision_sock, kp, ki, kd, dt):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot_id = robot_id
        self.vision_sock = vision_sock
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.v_max = 225
        self.v_min = 70
        self.v_linear = 225
        self.phi = 0
        
        self.last_speed_time = time.time()
        
        if COR_DO_TIME == 1:
            self._robot_attr = "robots_blue"
        elif COR_DO_TIME == 0:
            self._robot_attr = "robots_yellow"
        else: 
            raise ValueError(f"COR_DO_TIME: {COR_DO_TIME} é inválido, altere-o no 'config_ideal.py'.")
        
        self.kf = KalmanFilter(
            dt = self.dt,
            u_x = 0, u_y = 0,
            std_acc = 0.5,
            x_std_meas=0.01,
            y_std_meas=0.01
            )

        signal.signal(signal.SIGINT, self.off)
        signal.signal(signal.SIGTERM, self.off)
        
    def update_state(self):
        """
        Obtém os dados da visão, atualiza o Filtro de Kalman e retorna o estado estimado.
        Retorna (pos_filtrada, vel_filtrada, orientacao, bola_pos, outros_robos_pos)
        """
        data, _ = self.vision_sock.recvfrom(1024)
        frame = wr.SSL_WrapperPacket().FromString(data)
        
        # Posição da bola
        ball_pos = None
        if frame.detection.balls:
            ball_pos = (frame.detection.balls[0].x / 1000, frame.detection.balls[0].y / 1000)

        # Posição de outros robôs (obstáculos)
        obstacles = []
        for robot in frame.detection.robots_yellow:
            if robot.robot_id != self.robot_id:
                obstacles.append((robot.x / 1000, robot.y / 1000))
        for robot in frame.detection.robots_blue:
            if robot.robot_id != self.robot_id: # Se seu robô for azul, ajuste a lógica
                obstacles.append((robot.x / 1000, robot.y / 1000))

        # Encontra nosso robô e atualiza o filtro
        robots = getattr(frame.detection, COR_DO_TIME)
        for robot in robots:
            if robot.robot_id == self.robot_id:
                # Posição medida pela câmera (em metros)
                px_measured = robot.x / 1000
                py_measured = robot.y / 1000
                orientation = robot.orientation

                # --- Lógica do Filtro de Kalman ---
                self.kf.predict()
                # O filtro precisa da medição como um vetor coluna numpy
                measurement = np.array([[px_measured], [py_measured]])
                measurement = np.reshape(measurement, (2,1))
                try:
                    self.kf.update(measurement)
                
                except:
                    return None, None, None, None, None
                # O estado estimado é [px, py, vx, vy]
                filtered_pos = (self.kf.x[0, 0], self.kf.x[1, 0])
                filtered_vel = (self.kf.x[2, 0], self.kf.x[3, 0])

                return filtered_pos, filtered_vel, orientation, ball_pos, obstacles

        return None, None, None, None, None
    

    def speed_control(self, U, omega):

        vr = (2 * U + omega * 7.5) / 3
        vl = (2 * U - omega * 7.5) / 3
        vr = max(min(vr, self.v_max), self.v_min)
        vl = max(min(vl, self.v_max), self.v_min)
        
        if math.isnan(vr) or math.isnan(vl):
            vr, vl = 0, 0
        
        return int(vl), int(vr)


    def send_speed(self, speed_left, speed_right):
        direction_left = 1 if speed_left >= 0 else 0
        direction_right = 1 if speed_right >= 0 else 0
        combined_value = (abs(speed_left) << 24) | (abs(speed_right) << 16) | (direction_left << 8) | direction_right
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(combined_value.to_bytes(4, byteorder='little'))
        except Exception as e:
            print(f"Erro ao enviar dados: {e}")

    
    def follow_ball(self):
        a = 1
        interror_phi = Integral_part_phi = fant_phi = 0
        phi_obs = 0
        omega_ant = 0
        angulo_anterior_phid = 0
        angulo_anterior_phi_robot = 0
        
        x_ant = 0
        y_ant = 0
        

        while a == 1:
            state = self.update_state()[0:4]
            
            if state[0] is None or state[3] is None:
                time.sleep(dt)
                continue
            
            (x, y), (x_speed, y_speed), phi_obs, (ball_x, ball_y) = state

            if x is None or y is None:
                continue

            phid = math.atan2((ball_y - y), (ball_x - x))
            if phid > 3.15:
                phid = phid - 6.2832

            # Calcula la diferencia entre el ángulo actual y el anterior
            diferencia_phid = phid - angulo_anterior_phid
            diferencia_phi  = phi_obs - angulo_anterior_phi_robot
            # Si la diferencia es mayor que π, ajusta restando 2π
            if diferencia_phid > math.pi:
                phid -= 2 * math.pi
            # Si la diferencia es menor que -π, ajusta sumando 2π
            elif diferencia_phid < -math.pi:
                phid += 2 * math.pi
            
            # Si la diferencia es mayor que π, ajusta restando 2π
            if diferencia_phi > math.pi:
                phi_obs -= 2 * math.pi
            # Si la diferencia es menor que -π, ajusta sumando 2π
            elif diferencia_phi < -math.pi:
                phi_obs += 2 * math.pi
            
            # Actualiza el ángulo anterior
            angulo_anterior_phid = phid
            angulo_anterior_phi_robot = phi_obs

            error_phi = phid - phi_obs
            
            omega, fant_phi, interror_phi, Integral_part_phi = self.pid_controller(self.kp, self.ki, self.kd, self.dt, error_phi, interror_phi, fant_phi, Integral_part_phi)
            

            error_distance = math.sqrt((ball_y - y)**2 + (ball_x - x)**2)
            error_distance_global = math.sqrt((ball_y - y) ** 2 + (ball_x - x) ** 2)
            
            U = self.v_linear
            
            current_time = time.time()

            if current_time - self.last_speed_time >= self.dt:
                
                #Evitando travamentos em paredes
                
                if (abs(x_ant - x) <= 0.003 and abs(y_ant - y) <= 0.003):
                # if False:
                    self.travado(x, y, x_ant, y_ant)

                else:
                    vl, vr = self.speed_control(U, omega)
                    self.send_speed(vl + 30, vr)
                    
                self.last_speed_time = current_time
            
            x_ant = x
            y_ant = y       
    

    def pid_controller(self, kp, ki, kd, dt, error, interror, fant, Integral_part):
        Integral_saturation = 1
        raizes = math.sqrt(kd), math.sqrt(kp), math.sqrt(ki)
        Filter_e = 1 / (max(raizes) * 10)   
        unomenosalfaana = math.exp(-(dt / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror += error
        f = unomenosalfaana * fant + alfaana * error
        deerror = (f - fant) / dt if fant != 0 else f / dt
        Integral_part = min(max(Integral_part + ki * interror * dt, -Integral_saturation), Integral_saturation)
        PID = kp * error + Integral_part + deerror * kd
        return PID, f, interror, Integral_part
    

    def travado(self, x, y, x_ant, y_ant):
        current_time = time.time()
        x_ant = x_ant
        y_ant = y_ant
        x = x
        y = y
        a = 0
        try:
            while (abs(x_ant - x) <= 0.003 and abs(y_ant - y) <= 0.003):
                if (current_time - self.last_speed_time >= 0.1):
                   (x, y) = self.update_state()[0]

                   if (a % 2 == 0):
                       a += 1
                       self.send_speed(-210, -205)
                   else:
                       a += 1
                       self.send_speed(210, 205)

                   self.last_speed_time = current_time
                   current_time = time.time()
                   x_ant, y_ant = x, y
                else:
                   x_ant, y_ant = x, y
                   (x, y) = self.update_state[0]
                   current_time = time.time()
        except:
            return
        return


    def angle_diff(a, b):
        diff = b - a
        return
    
    
    def wrap_angle(angle):
        return (angle + math.pi) % (2*math.pi) - math.pi
    
    
    
    def off(self, signum=None, frame=None ):
        self.send_speed(0,0)
        sys.exit()
        return
    

if __name__ == "__main__":
    VISION_IP = "224.5.23.2"
    VISION_PORT = 10015
    ROBOT_IP = IP_ARES
    ROBOT_PORT = 80
    ROBOT_ID = ID_ARES

    Kp = 10
    Ki = 2
    Kd = 1.2
    dt = 0.1
    

    vision_sock = init_vision_socket(VISION_IP, VISION_PORT)
    crb01 = Corobeu(ROBOT_IP, ROBOT_PORT, ROBOT_ID, vision_sock, Kp, Ki, Kd, dt)

    crb01.follow_ball()