#!/bin/bash

echo "Iniciando os 3 robôs... Pressione CTRL+C nesta janela para parar todos."

# Array para guardar os PIDs (Process IDs) dos processos filhos
pids=()

# Função de limpeza que será chamada ao pressionar CTRL+C
cleanup() {
    echo -e "\n\nSinal de interrupção recebido, encerrando robôs..."
    # Mata cada processo filho usando o PID guardado
    for pid in "${pids[@]}"; do
        # O 'kill' envia um sinal de término (SIGTERM) para o processo
        kill "$pid"
    done
    # Espera um pouco para garantir que foram encerrados
    sleep 1
    echo "Robôs parados."
}

# O 'trap' intercepta os sinais e chama a função 'cleanup'
# SIGINT é o sinal do CTRL+C
# SIGTERM é um sinal de término comum
trap 'cleanup' SIGINT SIGTERM

# Inicia os processos e guarda seus PIDs
python3 CorobeuKRATOS_ball.py &
pids+=($!) # $! é o PID do último processo iniciado em background

python3 CorobeuZEUS_ball.py &
pids+=($!)

python3 CorobeuARES_ball.py &
pids+=($!)

echo "PIDs dos robôs: ${pids[*]}"
echo "Jogo em andamento. Aguardando CTRL+C para finalizar..."

# 'wait' ainda é importante para manter o script rodando e esperando
# O "wait -n" espera pelo próximo processo terminar, e sai com o status dele.
# Isso é mais robusto para scripts que podem terminar por conta própria.
wait -n

# Executa a limpeza caso um dos robôs pare por conta própria
cleanup