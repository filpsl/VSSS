import numpy as np
from queue import PriorityQueue

def calcular_forca_atrativa(pos_robo, pos_alvo, k_att):
    distancia = np.linalg.norm(pos_alvo - pos_robo)
    if distancia == 0:
        return np.zeros(2)
    # A força é um vetor unitário na direção do alvo, escalado por k_att e distância
    vetor_unitario = (pos_alvo - pos_robo) / distancia
    # A magnitude pode ser simplesmente proporcional à distância ou constante
    # Uma abordagem comum é ter uma força que aumenta com a distância até um certo limite
    magnitude = k_att * distancia 
    return vetor_unitario * magnitude


def calcular_forca_repulsiva(pos_robo, pos_obstaculo, raio_obstaculo, k_rep, raio_influencia):
    vetor_dist = pos_robo - pos_obstaculo
    distancia = np.linalg.norm(vetor_dist)
    
    # Se estiver fora da zona de influência do obstáculo, a força é zero
    if distancia > raio_influencia or distancia == 0:
        return np.zeros(2)
    
    # Se estiver dentro do raio do obstáculo (colisão iminente), força máxima
    if distancia <= raio_obstaculo:
        return (vetor_dist / distancia) * float('inf')

    # A força é inversamente proporcional à distância e aponta para longe do obstáculo
    vetor_unitario = vetor_dist / distancia
    magnitude = k_rep * (1.0/distancia - 1.0/raio_influencia) / (distancia**2)
    return vetor_unitario * magnitude


# Pseudocódigo simplificado da lógica principal do A*
# A implementação completa pode ser encontrada em [18]


def a_star_search(grid, start_pos, goal_pos):
    """
    Encontra o caminho mais curto de start_pos para goal_pos usando o algoritmo A*.
    """
    rows, cols = grid.shape
    open_list = PriorityQueue()
    open_list.put((0, start_pos)) # (f_cost, position)

    came_from = {} # Dicionário para reconstruir o caminho

    # g_cost: Custo do início até a posição atual.
    # Inicializa todos os custos como infinito, exceto o inicial.
    g_cost = { (r, c): float('inf') for r in range(rows) for c in range(cols) }
    g_cost[start_pos] = 0

    # f_cost: Custo total estimado (g_cost + heurística).
    f_cost = { (r, c): float('inf') for r in range(rows) for c in range(cols) }
    f_cost[start_pos] = heuristic(start_pos, goal_pos)
    
    # Um conjunto (set) para verificar rapidamente se um nó está na open_list
    open_list_hash = {start_pos}

    while not open_list.empty():
        # Pega o nó na fila de prioridade com o menor f_cost
        # O item é (f_cost, position), então pegamos a posição com [1]
        current_pos = open_list.get()[1]
        open_list_hash.remove(current_pos)

        if current_pos == goal_pos:
            return reconstruct_path(came_from, current_pos) # Sucesso!

        for neighbor_pos in get_valid_neighbors(grid, current_pos):
            # A distância de um nó para seu vizinho é 1 (cardinal) ou sqrt(2) (diagonal)
            # Para simplificar e garantir a admissibilidade da heurística, usamos o custo real.
            move_cost = heuristic(current_pos, neighbor_pos)
            tentative_g_cost = g_cost[current_pos] + move_cost
            
            if tentative_g_cost < g_cost[neighbor_pos]:
                # Este é um caminho melhor. Registre-o.
                came_from[neighbor_pos] = current_pos
                g_cost[neighbor_pos] = tentative_g_cost
                f_cost[neighbor_pos] = tentative_g_cost + heuristic(neighbor_pos, goal_pos)
                
                if neighbor_pos not in open_list_hash:
                    open_list.put((f_cost[neighbor_pos], neighbor_pos))
                    open_list_hash.add(neighbor_pos)
    
    return None # Caminho não encontrado
def heuristic(a, b):
    """Calcula a distância Euclidiana entre dois pontos (a e b)."""
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

def get_valid_neighbors(grid, node):
    """Obtém os vizinhos válidos de um nó no grid."""
    neighbors = []
    rows, cols = grid.shape
    
    # Movimentos possíveis (8 direções)
    for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
        r, c = node[0] + dr, node[1] + dc

        # Verifica se o vizinho está dentro dos limites do grid
        if 0 <= r < rows and 0 <= c < cols:
            # Verifica se não é um obstáculo (assumindo que 1 é obstáculo)
            if grid[r][c] == 0:
                neighbors.append((r, c))
    return neighbors


def reconstruct_path(came_from, current):
    """Reconstrói o caminho do início ao fim."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse() # Inverte para ter a ordem do início ao fim
    return path
