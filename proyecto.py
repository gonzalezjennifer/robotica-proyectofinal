import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random
import math
import copy

#Función para calcular la diferencia angular
def AngDif(carAngle, goalAngle):
    mag = math.acos(math.cos(carAngle) * math.cos(goalAngle) + math.sin(carAngle) * math.sin(goalAngle))
    dir = math.cos(carAngle) * math.sin(goalAngle) - math.sin(carAngle) * math.cos(goalAngle)
    return math.copysign(mag, dir)

#Función para calcular la distancia entre dos puntos
def calcular_distancia(x1, y1, x2, y2):
    distancia = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distancia

#Función para agregar un nuevo nodo al grafo
def agregar_nodo(grafo, id_nodo, id_padre, x, y, r, tipo):
    grafo.add_node(id_nodo, id_padre=id_padre, x=x, y=y, r=r, tipo=tipo)

#Función para agregar una nueva arista al grafo
def agregar_arista(grafo, id_nodo_origen, id_nodo_destino):
    grafo.add_edge(id_nodo_origen, id_nodo_destino)

#Función para mostrar todo el grafo
def dibujar_grafo(grafo, colores_por_nodo=None, texto_adicional=None):
    posiciones = {nodo: (datos['x'], datos['y']) for nodo, datos in grafo.nodes(data=True)}
    # Obtener colores para cada nodo, usando 'skyblue' si no se especifica un color
    colores = [colores_por_nodo.get(nodo, 'skyblue') for nodo in grafo.nodes]
    # Dibuja los nodos y las aristas
    nx.draw(grafo, pos=posiciones, with_labels=False, font_weight='bold', node_size=600, node_color=colores, font_size=8, cmap=plt.cm.Blues)
    # Agrega texto adicional a cada nodo
    if texto_adicional:
        labels = {nodo: f"{nodo}\n{texto_adicional.get(nodo, '')}" for nodo in grafo.nodes}
        nx.draw_networkx_labels(grafo, pos=posiciones, labels=labels, font_size=8)
    # Limita el rango de visualización
    plt.xlim(-2.5, 2.5)
    plt.ylim(-2.5, 2.5)
    # Muestra el grafo
    plt.show()

#Función para rotar los movimientos en base a la posición actual de auto
def rotacion(angulo):
    global movs
    #Para los 6 movimientos
    for i in range (1, 7):
        x_aux = movs[i][1]
        y_aux = movs[i][2]
        movs[i][1] = x_aux*np.cos(angulo) - y_aux*np.sin(angulo)
        movs[i][2] = x_aux*np.sin(angulo) + y_aux*np.cos(angulo)
    

#Movimientos primitivos 
movs = [
    [0, 0, 0, 0], #Detenido
    [1, 0.25, 0.0, 0.0], #Hacia delante
    [2, 0.2, 0.2, 0.5], #Hacia delante e izquierda
    [3, 0.2, -0.2, -0.5], #Hacia delante y derecha
    [4, -0.25, 0.0, 0.0], #Hacia atrás
    [5, -0.2, 0.2, -0.5], #Hacia atrás e izquierda 
    [6, -0.2, -0.2, 0.5] #Hacia atrás y derecha
]
movs_cp = copy.deepcopy(movs)

# Crear un grafo vacío
grafo = nx.DiGraph()

# Agregar un nodo raíz (inicio)
agregar_nodo(grafo, 0, None, -2, -2, np.pi/2, 0)

# Meta
meta = [2.0, 2.0, np.pi/2]

#Variables para buscar el nodomas cercano al punto aleatorio
id_nuevo_nodo = 1
best_hoja = 0

#Variable usada para mostrar en el grafo que tipo de movimiento es cada nodo
texto_adicional_nodos = {0: '0'}

#Número máximo de veces que se ejecutara en algoritmo de RRT
for _ in range(400):
    #Reinicio de error y punto aleatorio
    best_error = 1000
    x_rnd = random.uniform(-2.5, 2.5)
    y_rnd = random.uniform(-2.5, 2.5)

    #Recorrer todo el arreglo buscando el nodo mas cercano al punto aleatorio
    for nodo_id, datos_nodo in grafo.nodes(data=True):
        error_actual = calcular_distancia(datos_nodo.get('x'), datos_nodo.get('y'), x_rnd, y_rnd)
        if(error_actual < best_error):
            best_nodo = nodo_id
            best_error = error_actual
            x_best = datos_nodo.get('x')
            y_best = datos_nodo.get('y')
            r_best = datos_nodo.get('r')

    #Se rotan los movimientos en base a la posicion actual del vehiculo
    rotacion(r_best)
    

    #Se revisa si alguno de los 6 movimientos te acerca a el punto aleatorio
    for mov_i in range(1, 7):
        err_prox = calcular_distancia(x_best+movs[mov_i][1], y_best+movs[mov_i][2], x_rnd, y_rnd)
        if(err_prox < best_error and -2.5 <= x_best+movs[mov_i][1] <= 2.5 and -2.5 <= y_best+movs[mov_i][2] <= 2.5):
            agregar_nodo(grafo, id_nuevo_nodo, best_nodo, x_best+movs[mov_i][1], y_best+movs[mov_i][2], r_best+movs[mov_i][3], mov_i)
            agregar_arista(grafo, best_nodo, id_nuevo_nodo)
            texto_adicional_nodos[id_nuevo_nodo] = mov_i 
            id_nuevo_nodo += 1
    
    #Se reinicia la orientacion de movimientos
    movs = copy.deepcopy(movs_cp)


#Encontrar nodo mas cercano a la meta
best_error = 1000
for nodo_id, datos_nodo in grafo.nodes(data=True):
        error_actual = calcular_distancia(datos_nodo.get('x'), datos_nodo.get('y'), meta[0], meta[1]) + AngDif(datos_nodo.get('r'), meta[2])/3
        if(error_actual < best_error):
            best_nodo = nodo_id
            best_error = error_actual

nodo_meta = best_nodo
colores_nodos = {0: 'yellow'}
colores_nodos[best_nodo] = 'red'


movs2do = []
movs2do.append(grafo.nodes[nodo_meta]['tipo'])
while(True):
    actual_nodo = grafo.nodes[best_nodo]['id_padre']
    if(grafo.nodes[actual_nodo]['id_padre'] == None):
        break
    movs2do.append(grafo.nodes[actual_nodo]['tipo'])
    colores_nodos[actual_nodo] = 'green'
    best_nodo = actual_nodo

print("El nodo mas cercano a la meta es: ", nodo_meta)
print(f"La posicion final de carrito es ({grafo.nodes[nodo_meta]['x']}, {grafo.nodes[nodo_meta]['y']})")
print(f"Con una rotacion de: {grafo.nodes[nodo_meta]['r']} radianes")

Nmovs2do = len(movs2do)
print("Numero de movimientos por hacer: ", Nmovs2do)
movs2do = movs2do[::-1]
print("Secuencia de movimientos por hacer: ", movs2do)

# Dibujar grafo
dibujar_grafo(grafo, colores_nodos, texto_adicional_nodos)