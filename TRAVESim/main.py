import socket

from vssproto.simulation.command_pb2 import Command, Commands
from vssproto.simulation.common_pb2 import Frame
from vssproto.simulation.packet_pb2 import Environment, Packet

import socket
import time
import math
import struct
import signal
import sys

def init_vision_socket(VISION_IP="224.0.0.1", VISION_PORT=10002):
    sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_in.bind((VISION_IP, VISION_PORT))

    sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_out.connect(("127.0.0.1", 20012))
    return sock_in, sock_out

class Corobeu:
    def __init__(self, robot_port, robot_id, COR_DO_TIME, vision_sock, vision_sock_out, kp, ki, kd, dt, omega_max):
        self.robot_port = robot_port
        self.robot_id = robot_id
        self.vision_sock = vision_sock
        self.vision_sock_out = vision_sock_out
        
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
        
        self.v_max = 40
        self.v_min = 0
        self.v_linear = 40
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
        environment_data = Environment()
        environment_data.ParseFromString(data)
        robots = getattr(environment_data.frame, self._robot_attr)
        for robot in robots:
            if robot.robot_id == self.robot_id:
                return robot.x, robot.y, robot.orientation, environment_data.frame.ball.x, environment_data.frame.ball.y
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
        
            cmd_packet = Commands()
            cmd_packet.robot_commands.append(
                Command(
                    id=self.robot_id,
                    yellowteam=True,
                    wheel_left=speed_left,
                    wheel_right=speed_right,
                ),
            )
            
            packet = Packet()
            packet.cmd.CopyFrom(cmd_packet)

            self.vision_sock_out.send(packet.SerializeToString())

    
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
                self.send_speed(vl, vr)

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
        self.send_speed(0,0)
        sys.exit()
        return
    

if __name__ == "__main__":
    ROBOT_ID = 1
    ROBOT_PORT = 80
    COR_DO_TIME = 0
    
    Kp = 6
    Ki = 3.63
    Kd = 2.46
    
    dt = 0.1
    omega_max = 15
    
    # Kp = 10
    # Ki = 3.63
    # Kd = 2.46
    
    vision_sock, vision_sock_out = init_vision_socket()
    crb01 = Corobeu(ROBOT_PORT, ROBOT_ID, COR_DO_TIME, vision_sock, vision_sock_out, Kp, Ki, Kd, dt, omega_max)

    crb01.follow_ball()

# def robot_command(frame: Frame, yellowteam: bool) -> tuple[float, float]:  # noqa: ARG001, FBT001
#     return 100, -100


# def main(yellow_team: bool) -> None:  # noqa: FBT001
#     ###################################################################################
#     # Connection setup
#     ###################################################################################

#     sock_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock_in.bind(("224.0.0.1", 10002))

#     sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock_out.connect(("127.0.0.1", 20012))

#     ###################################################################################
#     # Loop start
#     ###################################################################################

#     print("Loop start")

#     try:
#         while True:
#             ###########################################################################
#             # Receive data from simulator
#             ###########################################################################

#             data, address = sock_in.recvfrom(1024)
#             print(f"Received {len(data)} bytes from {address}")

#             environment_data = Environment()
#             environment_data.ParseFromString(data)

#             print(environment_data)  # Mostra todos os campos de forma legível

            
#             ###########################################################################
#             # Process data from simulator
#             ###########################################################################

#             robot_left_wheel, robot_right_wheel = robot_command(
#                 environment_data.frame,
#                 yellowteam=yellow_team,
#             )
#             ###########################################################################
#             # Send commands to simulator
#             ###########################################################################

#             cmd_packet = Commands()
#             cmd_packet.robot_commands.append(
#                 Command(
#                     id=0,
#                     yellowteam=yellow_team,
#                     wheel_left=robot_left_wheel,
#                     wheel_right=robot_right_wheel,
#                 ),
#             )
            
#             packet = Packet()
#             packet.cmd.CopyFrom(cmd_packet)

#             sock_out.send(packet.SerializeToString())

#     except KeyboardInterrupt:
#         print("\nExiting...")
#     finally:
#         sock_out.close()
#         sock_in.close()


# if __name__ == "__main__":
#     main(yellow_team=True)
