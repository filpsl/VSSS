# Arquivo: vsss_main.py (O SEU CÓDIGO DO ROBÔ)

import signal
import time
import sys

# Variável global para controlar o loop
rodando = True

def parar_robo(signum, frame):
    """
    Esta função é chamada quando o sinal SIGINT (Ctrl+C) é recebido.
    """
    global rodando
    print("\n[VSSS] -> Sinal de parada (Ctrl+C) recebido!")
    print("[VSSS] -> Enviando velocidade (0, 0) para os motores...")
    # Aqui você colocaria o código real para parar os motores
    # ex: comm.send_speed(0, 0)
    print("[VSSS] -> Encerrando o processo do robô.")
    rodando = False
    # sys.exit(0) # Opcional: pode sair imediatamente se preferir

# Associa a função parar_robo ao sinal de Ctrl+C
signal.signal(signal.SIGINT, parar_robo)

print("[VSSS] -> Código do robô iniciado. Aguardando comandos...")
print("[VSSS] -> Para parar, este processo precisa receber um sinal SIGINT (Ctrl+C).")

# Loop principal do seu robô
contador = 0
while rodando:
    print(f"[VSSS] -> Loop de controle principal rodando... Ciclo {contador}")
    # Lógica do robô aqui...
    # calcular_posicao()
    # mover_robo()
    time.sleep(0.1)
    contador += 1

print("[VSSS] -> Loop principal finalizado.")