# Arquivo: controle_gestos.py (O SCRIPT DE CONTROLE)

import cv2
import mediapipe as mp
import time
import subprocess # Importa o módulo para gerenciar processos
import signal     # Importa o módulo de sinais
import os         # Usado para enviar sinais em diferentes sistemas

# --- Classe HandGestureDetector (a mesma versão robusta de antes) ---
# (Pode copiar a classe da resposta anterior, ela não muda)
class HandGestureDetector:
    def __init__(self, static_image_mode=False, max_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode, max_hands, 1, min_detection_confidence, min_tracking_confidence)
        self.mp_draw = mp.solutions.drawing_utils
        self.tip_ids = [4, 8, 12, 16, 20]
        self.pip_ids = [2, 6, 10, 14, 18]

    def find_hands(self, img, draw=True):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        if self.results.multi_hand_landmarks and draw:
            for hand_lms in self.results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(img, hand_lms, self.mp_hands.HAND_CONNECTIONS)
        return img

    def get_landmark_positions(self, img, hand_no=0):
        lm_list = []
        if self.results.multi_hand_landmarks:
            my_hand = self.results.multi_hand_landmarks[hand_no]
            for id, lm in enumerate(my_hand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lm_list.append([id, cx, cy])
        return lm_list

    def get_fingers_status(self, lm_list):
        if len(lm_list) == 0: return []
        fingers = []
        if lm_list[self.tip_ids[0]][1] > lm_list[self.tip_ids[0] - 1][1]:
            if lm_list[self.tip_ids[0]][1] > lm_list[self.pip_ids[0]][1]: fingers.append(1)
            else: fingers.append(0)
        else:
            if lm_list[self.tip_ids[0]][1] < lm_list[self.pip_ids[0]][1]: fingers.append(1)
            else: fingers.append(0)
        for id in range(1, 5):
            if lm_list[self.tip_ids[id]][2] < lm_list[self.pip_ids[id]][2]: fingers.append(1)
            else: fingers.append(0)
        return fingers

    def is_open_hand(self, fingers_status):
        return fingers_status == [1, 1, 1, 1, 1]

    def is_thumbs_up(self, fingers_status):
        return fingers_status == [1, 0, 0, 0, 0]


# --- Loop Principal de Controle (VERSÃO FINAL com SUBPROCESS) ---
def main():
    # IMPORTANTE: Coloque aqui o nome do seu script do robô
    NOME_DO_SCRIPT_VSSS = "Corobeu_ideal.py"
    # NOME_DO_SCRIPT_VSSS = "Corobeu_ideal.py"

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Erro: Não foi possível abrir a câmera.")
        return

    detector = HandGestureDetector()
    estado_programa = 'PARADO'
    vsss_process = None # Agora vamos armazenar o processo, não a thread

    print("="*50)
    print("Controle por Gestos VSSS - MODO PROCESSO ATIVADO")
    print(f"Alvo: '{NOME_DO_SCRIPT_VSSS}'")
    print("👍 Joinha: INICIA o processo do robô")
    print("🖐️ Mão Aberta: PARA o processo do robô (envia Ctrl+C)")
    print("="*50)

    try:
        while True:
            success, img = cap.read()
            if not success: break
            
            img = cv2.flip(img, 1)
            img = detector.find_hands(img)
            lm_list = detector.get_landmark_positions(img)

            if len(lm_list) != 0:
                fingers_status = detector.get_fingers_status(lm_list)
                if fingers_status:
                    joinha = detector.is_thumbs_up(fingers_status)
                    mao_aberta = detector.is_open_hand(fingers_status)

                    # INICIA o processo do robô
                    if joinha and estado_programa == 'PARADO':
                        print(">> Gesto JOINHA 👍: Iniciando processo do robô...")
                        # Inicia o script VSSS como um novo processo
                        vsss_process = subprocess.Popen(["python", NOME_DO_SCRIPT_VSSS])
                        estado_programa = 'RODANDO'
                        time.sleep(2)

                    # PARA o processo do robô (enviando Ctrl+C)
                    elif mao_aberta and estado_programa == 'RODANDO':
                        print(">> Gesto MÃO ABERTA 🖐️: Enviando sinal de parada (Ctrl+C) ao processo do robô...")
                        # Envia o sinal SIGINT (equivalente ao Ctrl+C)
                        vsss_process.send_signal(signal.SIGINT)
                        # vsss_process.wait() # Espera o processo terminar
                        vsss_process = None
                        estado_programa = 'PARADO'
                        print(">> Processo do robô parado. Aguardando novo comando...")
                        time.sleep(2)
                        
            # Verifica se o processo terminou por conta própria
            if vsss_process and vsss_process.poll() is not None:
                print(">> Processo do robô terminou inesperadamente.")
                vsss_process = None
                estado_programa = 'PARADO'

            status_color = (0, 255, 0) if estado_programa == 'RODANDO' else (0, 0, 255)
            cv2.putText(img, f"ESTADO: {estado_programa}", (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, status_color, 3)
            cv2.imshow("Controle por Gestos VSSS", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("\nPrograma de controle encerrado pelo usuário.")
    finally:
        # Rotina de limpeza final: garante que o processo filho seja parado
        if vsss_process and vsss_process.poll() is None:
            print("Limpando... Enviando sinal de parada final ao processo do robô...")
            vsss_process.send_signal(signal.SIGINT)
            vsss_process.wait()
        
        cap.release()
        cv2.destroyAllWindows()
        print("Recursos liberados. Até a próxima!")

if __name__ == "__main__":
    main()