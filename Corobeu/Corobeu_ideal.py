import socket
import time
import math
import struct
import signal
from configs import wrapper_pb2 as wr
import sys
from configs.config import IP_KRATOS, ID_KRATOS, COR_DO_TIME

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
    def __init__(self, robot_ip, robot_port, robot_id, vision_sock, kp, ki, kd, dt, omega_max):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot_id = robot_id
        self.vision_sock = vision_sock
        
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
        
        self.   v_max = 225
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
        robots = getattr(frame.detection, self._robot_attr)
        for robot in robots:
            if robot.robot_id == self.robot_id:
                return robot.x / 1000, robot.y / 1000, robot.orientation, frame.detection.balls[0].x / 1000, frame.detection.balls[0].y / 1000
        return None, None, None, None, None
    

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
        phi_obs = 0

        while True:

            x, y, phi_obs, ball_x, ball_y = self.get_position()

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
                self.send_speed(vl + 25, vr)

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
                self.send_speed(vl + 25, vr)
                
                self.last_speed_time = current_time
            
            if (error_distance <= 0.07):
                self.send_speed(0,0)
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
        provisional_output = proportional_term + self.ki * potential_integral + derivative_term
        if self.omega_min < provisional_output < self.omega_max:
            self.integral = potential_integral
            
        integral_term = self.ki * self.integral
        
        # --- 4. Soma Final e Saturação da Saída ---
        # Combina os três termos para obter a saída final do controlador.
        output = proportional_term + integral_term + derivative_term
        
        # Garante que a saída final NUNCA exceda os limites definidos.
        output = max(min(output, self.omega_max), self.omega_min)
        
        # --- 5. Atualização das Variáveis de Estado para a Próxima Iteração ---
        self.previous_error = error
        self.filtered_previous_error = filtered_error
        return output
    
    
    def wrap_angle(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi
    
    
    def off(self, signum=None, frame=None ):
        self.send_speed(0,0)
        sys.exit()
        return
    

if __name__ == "__main__":
    VISION_IP = "224.5.23.2"
    VISION_PORT = 10015
    ROBOT_IP = IP_KRATOS
    ROBOT_ID = ID_KRATOS
    ROBOT_PORT = 80
    
    Kp = 8
    Ki = 3.63
    Kd = 2.46
    
    dt = 0.1
    
    omega_max = 15
    
    # Kp = 10
    # Ki = 3.63
    # Kd = 2.46
    
    vision_sock = init_vision_socket(VISION_IP, VISION_PORT)
    crb01 = Corobeu(ROBOT_IP, ROBOT_PORT, ROBOT_ID, vision_sock, Kp, Ki, Kd, dt, omega_max)

    crb01.follow_path(0.4, 0.4)