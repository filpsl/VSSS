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

    
# --- Configurações da Simulação ---
dt = 0.1  # Intervalo de tempo entre os passos (segundos), 10 Hz
total_time = 20 # Tempo total da simulação (segundos)
num_steps = int(total_time / dt)

# Parâmetros do Filtro de Kalman
# u_x, u_y: aceleração de controle, 0 se não estiver controlando o objeto diretamente
u_x = 0
u_y = 0
# std_acc: incerteza sobre o modelo de movimento (ruído do processo)
# Se o objeto se move de forma um pouco errática, aumente este valor.
std_acc = 0.5 # Aceleração (m/s^2) - ajuste conforme o comportamento esperado do objeto
# x_std_meas, y_std_meas: incerteza das medições do sensor (ruído da medição)
# Se o sensor for impreciso, aumente estes valores.
x_std_meas = 2.0 # Posição em X (unidade, e.g., pixels, cm, metros)
y_std_meas = 2.0 # Posição em Y (unidade, e.g., pixels, cm, metros)

# --- Inicialização do Filtro de Kalman ---
kf = KalmanFilter(dt, u_x, u_y, std_acc, x_std_meas, y_std_meas)

# Opcional: Definir estado inicial do filtro se você tiver uma primeira estimativa
# kf.x = np.array([[0.], [0.], [0.], [0.]]) # [px, py, vx, vy] - Começa em (0,0) com velocidade 0

# --- Gerando o Movimento Real (Ground Truth) ---
# Vamos simular um objeto começando em (10, 20) com velocidade (5, 3)
true_px = 10.0
true_py = 20.0
true_vx = 7.0
true_vy = 3.0

# Listas para armazenar os dados para plotagem (opcional, mas muito útil)
true_positions = []
measured_positions = []
estimated_positions = []

print("Iniciando simulação...")
print(f"Estado inicial real: Px={true_px:.2f}, Py={true_py:.2f}, Vx={true_vx:.2f}, Vy={true_vy:.2f}")

# --- Loop de Simulação ---
for i in range(num_steps):
    # 1. Atualizar o movimento real (Ground Truth)
    # Movimento constante (vx, vy não mudam, apenas px, py)
    true_px += true_vx * dt
    true_py += true_vy * dt

    # Poderíamos adicionar uma aceleração real aqui para testar:
    # true_vx += true_accel_x * dt
    # true_vy += true_accel_y * dt

    true_positions.append((true_px, true_py))

    # 2. Gerar Medição Ruidosa
    # Adicionando ruído gaussiano às posições reais
    measurement_noise_x = np.random.normal(0, x_std_meas)
    measurement_noise_y = np.random.normal(0, y_std_meas)

    measured_px = true_px + measurement_noise_x
    measured_py = true_py + measurement_noise_y
    
    # A medição para o filtro de Kalman deve ser um vetor coluna
    z = np.array([[measured_px], [measured_py]])
    measured_positions.append((measured_px, measured_py))

    # 3. Executar o Filtro de Kalman
    kf.predict()  # Etapa de Predição
    kf.update(z)  # Etapa de Atualização

    estimated_positions.append((kf.x[0, 0], kf.x[1, 0]))

    # Opcional: Imprimir estados para ver o progresso
    if i % 10 == 0: # Imprime a cada 10 passos
        print(f"\n--- Passo {i*dt:.1f}s ---")
        print(f"Real:      Px={true_px:.2f}, Py={true_py:.2f}")
        print(f"Medido:    Px={measured_px:.2f}, Py={measured_py:.2f}")
        print(f"Estimado:  Px={kf.x[0,0]:.2f}, Py={kf.x[1,0]:.2f}, Vx={kf.x[2,0]:.2f}, Vy={kf.x[3,0]:.2f}")
        # print("Pós-atualização P:\n", kf.P) # Descomente para ver a matriz de covariância

print("\nSimulação concluída.")

# --- Opcional: Visualizar os Resultados ---
try:
    import matplotlib.pyplot as plt

    true_x = [p[0] for p in true_positions]
    true_y = [p[1] for p in true_positions]
    measured_x = [p[0] for p in measured_positions]
    measured_y = [p[1] for p in measured_positions]
    estimated_x = [p[0] for p in estimated_positions]
    estimated_y = [p[1] for p in estimated_positions]

    plt.figure(figsize=(10, 8))
    plt.plot(true_x, true_y, 'g-', label='Posição Real (Ground Truth)')
    plt.plot(measured_x, measured_y, 'rx', label='Medições Ruidosas', alpha=0.6)
    plt.plot(estimated_x, estimated_y, 'b--', label='Estimativa do Kalman')
    plt.xlabel('Posição X')
    plt.ylabel('Posição Y')
    plt.title('Simulação do Filtro de Kalman')
    plt.legend()
    plt.grid(True)
    plt.axis('equal') # Garante que as escalas X e Y sejam iguais
    plt.show()

except ImportError:
    print("\nMatplotlib não encontrado. Para visualizar os resultados, instale-o com 'pip install matplotlib'.")
