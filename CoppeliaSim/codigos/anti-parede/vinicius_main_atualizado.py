import time
import math
import sims.sim as sim
import signal
import sys

# CONSTANTES GLOBAIS

# Dimensões do campo (AJUSTE ESTES VALORES COM OS DA SUA CENA!)
FIELD_X_MAX = 0.75
FIELD_X_MIN = -0.75
FIELD_Y_MAX = 0.65
FIELD_Y_MIN = -0.65

# Parâmetros da Estratégia
# Margem de segurança. O robô ativará a fuga quando chegar a esta distância da parede.
WALL_MARGIN = 0.10 # 10 cm

# Nomes dos Estados da Máquina de Estados 
STATE_ATTACKING_BALL = "attacking_ball"
STATE_AVOIDING_WALL = "avoiding_wall"


# FUNÇÃO DE CONEXÃO (pode ser mantida fora da classe
def connect_CRB(port):
    """
    Função usada para se comunicar com o CoppeliaSim
    """
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID != -1:
        print("Conectado ao CoppeliaSim na porta", port)
    else:
        print("Falha ao conectar na porta", port)
        sys.exit()

    # Pega os handles (identificadores) dos objetos na cena
    returnCode, robot = sim.simxGetObjectHandle(clientID, 'robot01', sim.simx_opmode_blocking)
    returnCode, motorE = sim.simxGetObjectHandle(clientID, 'motorL01', sim.simx_opmode_blocking)
    returnCode, motorD = sim.simxGetObjectHandle(clientID, 'motorR01', sim.simx_opmode_blocking)
    returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking)
    
    return clientID, robot, motorE, motorD, ball

#CLASSE PRINCIPAL DO ROBÔ
class Corobeu:
    def __init__(self, kp, ki, kd, dt, omega_max):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        # --- Limites de Saída (para anti-windup e segurança) ---
        self.omega_min = -omega_max
        self.omega_max = omega_max
        
        # --- Filtro para o Termo Derivativo ---
        self.filter_alpha = 0.5 
        
        # --- Variáveis de Estado do PID (precisam ser lembradas entre as chamadas) ---
        self.integral = 0.0
        self.previous_error = 0.0
        self.filtered_previous_error = 0.0
        
        # --- Parâmetros Físicos e de Movimento do Robô ---
        self.v_max = 8
        self.v_min = -8
        self.v_linear = 6
        
        # --- Variáveis da Máquina de Estados ---
        self.current_state = STATE_ATTACKING_BALL
        self.robot_position = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.ball_position = {'x': 0.0, 'y': 0.0}
        
        # --- Temporizador para o loop de controle ---
        self.last_update_time = time.time()
        
        # --- Configura o desligamento seguro ---
        signal.signal(signal.SIGINT, self.off)
        signal.signal(signal.SIGTERM, self.off)

    def speed_control(self, U, omega):
        """
        Calcula a velocidade de cada roda e escala para não passar do limite máximo,
        mantendo a proporção de giro.
        """
        vr = (2 * U + omega * 7.5) / 3.0 # Roda Direita (Right)
        vl = (2 * U - omega * 7.5) / 3.0 # Roda Esquerda (Left)
        
        # Encontra o fator de escala necessário
        max_speed = max(abs(vr), abs(vl))
        if max_speed > self.v_max:
            scale_factor = self.v_max / max_speed
            vr *= scale_factor
            vl *= scale_factor
        
        if math.isnan(vr) or math.isnan(vl):
            vr, vl = 0, 0
        
        return int(vl), int(vr)

    def pid_controller(self, error):
        """
        Controlador PID robusto com filtro na derivada e anti-windup na integral.
        """
        # --- 1. Termo Proporcional (P) ---
        proportional_term = self.kp * error
        
        # --- 2. Termo Derivativo (D) com Filtro ---
        filtered_error = (self.filter_alpha * error) + (1 - self.filter_alpha) * self.filtered_previous_error
        derivative_term = self.kd * (filtered_error - self.filtered_previous_error) / self.dt
        
        # --- 3. Termo Integral (I) com Anti-Windup Condicional ---
        potential_integral = self.integral + error * self.dt
        provisional_omega = proportional_term + self.ki * potential_integral + derivative_term
        
        if self.omega_min < provisional_omega < self.omega_max:
            self.integral = potential_integral
            
        integral_term = self.ki * self.integral
        
        # --- 4. Soma Final e Saturação da Saída ---
        omega = proportional_term + integral_term + derivative_term
        omega = max(min(omega, self.omega_max), self.omega_min)
        
        # --- 5. Atualização das Variáveis de Estado para a Próxima Iteração ---
        self.previous_error = error
        self.filtered_previous_error = filtered_error
        
        return omega
    
    # --- NOVOS MÉTODOS PARA A ESTRATÉGIA ---
    
    def is_near_wall(self):
        """Verifica se o robô está na margem de perigo perto de uma parede."""
        x = self.robot_position['x']
        y = self.robot_position['y']
        
        if (x > FIELD_X_MAX - WALL_MARGIN or
            x < FIELD_X_MIN + WALL_MARGIN or
            y > FIELD_Y_MAX - WALL_MARGIN or
            y < FIELD_Y_MIN + WALL_MARGIN):
            return True
        return False

    def perform_wall_avoidance(self):
        """
        Calcula as velocidades para o robô se afastar da parede e virar para o centro.
        Retorna (vl, vr).
        """
        robot_x = self.robot_position['x']
        robot_y = self.robot_position['y']
        robot_phi = self.robot_position['phi']

        # 1. Mira no centro do campo (0,0) para se afastar da parede
        target_angle_to_center = math.atan2(-robot_y, -robot_x)
        
        # 2. Calcula o erro de ângulo
        error_phi = self.wrap_angle(target_angle_to_center - robot_phi)
        
        # 3. Usa um controlador Proporcional simples para girar rápido
        kp_avoid = 5.0 
        omega = kp_avoid * error_phi
        
        # 4. Define uma velocidade linear para trás para se afastar
        U = -self.v_linear / 2 
        
        # Se o robô já está virado para o centro, ele para de dar ré e apenas gira
        if abs(error_phi) < math.radians(45): # 45 graus
            U = 0

        print(f"--- EVITANDO PAREDE --- Ângulo alvo: {math.degrees(target_angle_to_center):.1f} deg, Erro: {math.degrees(error_phi):.1f} deg")
        
        return self.speed_control(U, omega)

    # --- FUNÇÕES UTILITÁRIAS ---

    def wrap_angle(self, angle):
        """Garante que um ângulo esteja no intervalo de -pi a pi."""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def off(self, signum=None, frame=None):
        """Função para desligar o robô de forma segura."""
        print("Desligando o robô...")
        # Adicionar aqui a lógica para parar os motores no simulador se necessário
        sys.exit(0)

    # --- LOOP PRINCIPAL (ANTIGO 'follow_ball') ---

    def run_strategy(self):
        """
        Loop principal que executa a máquina de estados do robô.
        """
        (clientID, robot, motorE, motorD, ball) = connect_CRB(19995)
        
        print("Estratégia iniciada. Pressione Ctrl+C para parar.")
        
        # Loop principal de estratégia
        while True:
            current_time = time.time()
            if (current_time - self.last_update_time) < self.dt:
                continue # Garante que o loop rode na frequência definida por dt
            self.last_update_time = current_time

            # 1. ATUALIZAR DADOS DO MUNDO
            s, robotPos = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
            s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
            s, robotOri = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
            
            if robotPos and robotOri:
                self.robot_position['x'] = robotPos[0]
                self.robot_position['y'] = robotPos[1]
                # A orientação Z (gamma) é a que nos interessa para rotação no plano
                # O ajuste de -pi/2 pode ser necessário dependendo de como o robô foi modelado
                self.robot_position['phi'] = self.wrap_angle(robotOri[2] - math.pi/2)
            
            if ballPos:
                self.ball_position['x'] = ballPos[0]
                self.ball_position['y'] = ballPos[1]

            # 2. VERIFICAR TRANSIÇÕES DE ESTADO
            if self.is_near_wall():
                self.current_state = STATE_AVOIDING_WALL
            else:
                self.current_state = STATE_ATTACKING_BALL

            # 3. EXECUTAR A LÓGICA DO ESTADO ATUAL
            if self.current_state == STATE_ATTACKING_BALL:
                print(f"ESTADO: [ATACANDO BOLA]")
                phid = math.atan2(self.ball_position['y'] - self.robot_position['y'], 
                                  self.ball_position['x'] - self.robot_position['x'])
                error_phi = self.wrap_angle(phid - self.robot_position['phi'])
                omega = self.pid_controller(error_phi)
                vl, vr = self.speed_control(self.v_linear, omega)
            
            elif self.current_state == STATE_AVOIDING_WALL:
                print(f"ESTADO: [EVITANDO PAREDE]")
                vl, vr = self.perform_wall_avoidance()
            
            else: # Estado desconhecido
                vl, vr = 0, 0

            # 4. ENVIAR COMANDOS AOS MOTORES
            sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, vr, sim.simx_opmode_blocking)

# PONTO DE ENTRADA DO PROGRAMA
if __name__ == "__main__":
    
    kp = 3.0528502  
    kd = 0.79546531
    ki = 0
    
    dt = 0.05  # Tempo de ciclo do controlador (50 ms)
    omega_max = 8 # Velocidade angular máxima (rad/s)
    
    # --- Inicialização e Execução ---
    crb01 = Corobeu(kp, ki, kd, dt, omega_max)
    crb01.run_strategy()
