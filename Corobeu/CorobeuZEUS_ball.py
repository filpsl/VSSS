import socket
import time
import math
import numpy as np
import struct
import signal
from configs import wrapper_pb2 as wr
import sys
from configs.config import IP_ZEUS, ID_ZEUS, COR_DO_TIME

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
    def __init__(self, robot_ip, robot_port, robot_id, vision_sock, kp, ki, kd):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot_id = robot_id
        self.vision_sock = vision_sock
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
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
        
        signal.signal(signal.SIGINT, self.off)
        signal.signal(signal.SIGTERM, self.off)
        
    def get_position(self):
        data, _ = self.vision_sock.recvfrom(1024)
        frame = wr.SSL_WrapperPacket().FromString(data)
        robots = getattr(frame.detection, COR_DO_TIME)
        for robot in robots:
            if robot.robot_id == self.robot_id:
                return robot.x / 1000, robot.y / 1000, robot.orientation, frame.detection.balls[0].x / 1000, frame.detection.balls[0].y / 1000
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
        deltaT = 0.1
        a = 1
        interror_phi = Integral_part_phi = fant_phi = 0
        phi_obs = 0
        omega_ant = 0
        angulo_anterior_phid = 0
        angulo_anterior_phi_robot = 0
        
        x_ant = 0
        y_ant = 0
        

        while a == 1:
            x, y, phi_obs, ball_x, ball_y = self.get_position()

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
            omega, fant_phi, interror_phi, Integral_part_phi = self.pid_controller(self.kp, self.ki, self.kd, deltaT, error_phi, interror_phi, fant_phi, Integral_part_phi)
            

            error_distance = math.sqrt((ball_y - y)**2 + (ball_x - x)**2)
            error_distance_global = math.sqrt((ball_y - y) ** 2 + (ball_x - x) ** 2)
            
            U = self.v_linear
            
            current_time = time.time()

            if current_time - self.last_speed_time >= deltaT:
                
                #Evitando travamentos em paredes
                
                if (abs(x_ant - x) <= 0.003 and abs(y_ant - y) <= 0.003):
                    self.travado(x, y, x_ant, y_ant)

                else:
                    vl, vr = self.speed_control(U, omega)
                    self.send_speed(vl, vr)

                self.last_speed_time = current_time
            
            x_ant = x
            y_ant = y
            

    def pid_controller(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):
        Integral_saturation = 1
        raizes = math.sqrt(kd), math.sqrt(kp), math.sqrt(ki)
        Filter_e = 1 / (max(raizes) * 10)   
        unomenosalfaana = math.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror += error
        f = unomenosalfaana * fant + alfaana * error
        deerror = (f - fant) / deltaT if fant != 0 else f / deltaT
        Integral_part = min(max(Integral_part + ki * interror * deltaT, -Integral_saturation), Integral_saturation)
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
                   x, y = self.get_position()[0:2]

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
                   x, y = self.get_position()[0:2]
                   current_time = time.time()
        except:
            return
        return

    
    def off(self, signum=None, frame=None ):
        self.send_speed(0,0)
        sys.exit()
        return
    

if __name__ == "__main__":
    VISION_IP = "224.5.23.2"
    VISION_PORT = 10015
    ROBOT_IP = IP_ZEUS
    ROBOT_PORT = ID_ZEUS
    Kp = 10
    Ki = 2
    Kd = 1.2
    

    vision_sock = init_vision_socket(VISION_IP, VISION_PORT)
    crb01 = Corobeu(ROBOT_IP, ROBOT_PORT, ID_ZEUS, vision_sock, Kp, Ki, Kd)

    crb01.follow_ball()