import time
import math
import sims.sim as sim
import numpy as np
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


def algoritmo_goa(S, N, iter_max):
    
    # DEFINIÇÃO DO ESPAÇO DE BUSCA
    limite_superior = 8
    limite_inferior = -8
    
    # PARÂMETROS DO ALGORITMO
    coeficiente_exploracao = 0.05
    coeficiente_refinamento = 0.5
    coeficiente_pulo = 0.3
    probabilidade_pulo = 0.1
    
    parasita_porcentagem = 0.2
    num_parasitas = math.ceil(parasita_porcentagem * S)
    
    kp_max = 8
    ki_max = 3
    kd_max = 0.5
    
    # INICIALIZANDO OS PID
    coluna1 = np.random.rand(S) * 5 + 3
    coluna2 = np.random.rand(S) * ki_max
    coluna3 = np.random.rand(S) * kd_max
    posicoes = np.hstack((coluna1.reshape(-1, 1),
                   coluna2.reshape(-1, 1),
                   coluna3.reshape(-1, 1)))
    
    # INICIALIZANDO O ROBÔ
    crb01 = Corobeu(0.05)
    sim.simxSetObjectPosition(crb01.clientID, crb01.robot, -1, [-0.4, -0.4, 0.01], sim.simx_opmode_oneshot)    
    
    
    erros = [0.0] * S
    for i in range(S):
        erros[i] = fitnessFunc(crb01, posicoes[i])
        sim.simxSetObjectPosition(crb01.clientID, crb01.robot, -1, [-0.4, -0.4, 0.01], sim.simx_opmode_oneshot)
        print(f"Partícula[{i}]: {posicoes[i]} - Erro: {erros[i]}\n")
    
    melhor_indice_global = np.argmin(erros)
    melhor_posicao_global = posicoes[melhor_indice_global].copy()
    menor_erro_global = erros[melhor_indice_global]
    
    for iteracao in range(iter_max):
        for i in range(S):
            
            # RECOLHENDO UMA CABRA PARA ANÁLISE
            cabra_atual = posicoes[i].copy()
            
            # MOVIMENTANDO AS CABRAS EM QUALQUER DIREÇÃO
            direcao_aleatoria = np.random.uniform(limite_inferior, limite_superior, N)
            cabra_atual = cabra_atual + coeficiente_exploracao * np.random.rand() * direcao_aleatoria
            
            # MOVIMENTANDO AS CABRAS EM DIREÇÃO À MELHOR CABRA DA ITERAÇÃO ANTERIOR
            cabra_atual = cabra_atual + coeficiente_refinamento * (melhor_posicao_global - cabra_atual)
            
            # TALVEZ ELA DÊ UM PULO EM DIREÇÃO A UMA CABRA ALEATÓRIA
            if np.random.rand() <= probabilidade_pulo:
                indice_aleatorio = np.random.randint(0, S)
                
                while indice_aleatorio == i:
                    indice_aleatorio = np.random.randint(0, S)
                
                cabra_aleatoria = posicoes[indice_aleatorio]    
                cabra_atual = cabra_atual + coeficiente_pulo * (cabra_aleatoria - cabra_atual)
            
            # GARANTINDO QUE A CABRA NÃO SAIA DO ESPAÇO DE BUSCA
            cabra_atual = np.clip(cabra_atual, limite_inferior, limite_superior)


        for i in range(S):
            cabra = posicoes[i]
            erro_cabra = fitnessFunc(crb01, cabra)
            sim.simxSetObjectPosition(crb01.clientID, crb01.robot, -1, [-0.4, -0.4, 0.01], sim.simx_opmode_oneshot)
            print(f"Partícula[{i}]: {cabra} - Erro: {erro_cabra} - Menor Erro: {menor_erro_global} - Melhor partícula {melhor_posicao_global}\n")
            
            if erro_cabra < erros[i]:               # NO DOCUMENTO NÃO FICA CLARO SE 
                posicoes[i] = cabra_atual           # A MOVIMENTAÇÃO DEVE SER FEITA SEMPRE
                erros[i] = erro_cabra               # OU APENAS QUANDO MELHORA O ERRO.
        
        # SUBSTITUINDO AS PIORES CABRAS POR NOVAS CABRAS ALEATÓRIAS
        indice_parasitas = np.argsort(erros)[-num_parasitas:]
        for parasita_indice in indice_parasitas:
            posicoes[parasita_indice] = np.random.uniform(limite_inferior, limite_superior, N)
            erros[parasita_indice] = fitnessFunc(crb01, posicoes[parasita_indice])
            
        # ENCONTRANDO A MELHOR CABRA DESTA ITERAÇÃO
        melhor_indice_iteracao = np.argmin(erros)
        
        # ATUALIZANDO A MELHOR GLOBAL SE A MELHOR DESTA ITERAÇÃO FOR MELHOR
        if erros[melhor_indice_iteracao] < menor_erro_global:
            menor_erro_global = erros[melhor_indice_iteracao]
            melhor_posicao_global = posicoes[melhor_indice_iteracao].copy()
        
        iteracao += 1
        sim.simxSetObjectPosition(crb01.clientID, crb01.robot, -1, [-0.4, -0.4, 0.01], sim.simx_opmode_oneshot)
    return 


def fitnessFunc(crb01, particula):
    erro = crb01.cacar_pid(particula)
    return erro


class Corobeu:
    def __init__(self, dt):

        self.dt = dt
        self.checkpoint_dt = 2

        self.integral_range = 30
        self.interror = [0 for _ in range(self.integral_range)]
        self.Integral_part = 0  
        self.f_ant = 0        
        
        self.v_max = 8
        self.v_min = -8
        self.v_linear = 8
        self.phi = 0
        
        self.last_speed_time = time.time()
        self.last_checkpoint_time = time.time()
        
        # --- Conexão com o CoppeliaSim --- 
        self.clientID = 0;
        self.robot = 0;
        self.motorE = 0;
        self.motorD = 0;
        
        (self.clientID, self.robot, self.motorE, self.motorD, _) = connect_CRB(19995)
    

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


    def cacar_pid(self, PID):
        
        integral_counter = 0
        kp, ki, kd = PID[0], PID[1], PID[2]
        path = [[0.4, 0.4],
                [-0.4, 0.4],
                [-0.4, -0.4],
                [0.4, -0.4]]

        # LÓGICA 1: Inicializa o erro total para todo o percurso
        total_error = 0

        if (sim.simxGetConnectionId(self.clientID) != -1):

            sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)

            # LÓGICA 2: Loop externo para percorrer cada ponto no 'path'
            for target_point in path:
                # Define o alvo ATUAL para o loop de movimento
                path_x, path_y = target_point[0], target_point[1]

                # Reseta o acumulador de erro e o tempo para CADA novo alvo
                error_phi_sum = 0
                start_time = time.time()

                # Este loop 'while' agora é responsável por ir para UM único alvo
                while True:
                    
                    current_time = time.time()    
                    if current_time - self.last_speed_time < self.dt:
                        continue      
                                 
                    s, robotPosition = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    s, phi = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_blocking)

                    # Validação para evitar erros se a simulação não retornar posição
                    if not robotPosition:
                        continue 

                    x = robotPosition[0]
                    y = robotPosition[1]
                    phi_obs = phi[2] - math.pi/2

                    phid = math.atan2((path_y - y), (path_x - x))
                    phid = self.wrap_angle(phid)

                    phi_obs = self.wrap_angle(phi_obs)
                    error_phi = self.wrap_angle(phid - phi_obs)

                    error_phi_sum += abs(error_phi)

                    omega = self.pid_controller(kp, ki, kd, error_phi, integral_counter)

                    error_distance = math.sqrt((path_y - y)**2 + (path_x - x)**2)

                    U = self.v_linear  
                    
                    vl, vr = self.speed_control(U, omega)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorE, vl, sim.simx_opmode_blocking)    
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorD, vr, sim.simx_opmode_blocking)
                    self.last_speed_time = current_time

                    
                    integral_counter += 1
                    if integral_counter >= self.integral_range:
                        integral_counter = 0
                    
                    # LÓGICA 3: Se chegou ao alvo, quebra o loop 'while' para ir para o próximo ponto
                    if (error_distance < 0.07):
                        break
                    
                    # LÓGICA 4: Se o tempo esgotou, também quebra o loop
                    if current_time - start_time >= 5:
                        # Opcional: Penalizar o erro total se o tempo esgotou
                        error_phi_sum += 1000 # Adiciona uma grande penalidade
                        break
                    
                # LÓGICA 5: Acumula o erro do trecho recém-concluído no erro total
                total_error += error_phi_sum

        # LÓGICA 6: Retorna o erro acumulado de todo o percurso
        return total_error
            

    def pid_controller(self, kp, ki, kd, error, integral_counter):
        
        Integral_saturation = 5
        raizes = math.sqrt(kd), math.sqrt(kp), math.sqrt(ki)
        Filter_e = 1 / (max(raizes) * 10)   
        unomenosalfaana = math.exp(-(self.dt / Filter_e))
        alfaana = 1 - unomenosalfaana
        self.interror[integral_counter] = error
        f = unomenosalfaana * self.f_ant + alfaana * error
        deerror = (f - self.f_ant) / self.dt if self.f_ant != 0 else f / self.dt
        self.Integral_part = min(max(self.Integral_part + ki * sum(self.interror) * self.dt, -Integral_saturation), Integral_saturation)
        self.f_ant = f
        PID = kp * error + self.Integral_part + deerror * kd
        return PID
    
    
    def wrap_angle(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi
    
    
    def off(self, signum=None, frame=None ):
        
        sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)    
        sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)
        return
    

if __name__ == "__main__":

    algoritmo_goa(5, 3, 1000)