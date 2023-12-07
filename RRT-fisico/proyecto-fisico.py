#! /usr/bin/env python3
import rospy
import time
import tf
import math
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import random
import copy
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand

#Función para calcular la diferencia angular
def AngDif(carAngle, goalAngle):
    mag = math.acos(math.cos(carAngle) * math.cos(goalAngle) + math.sin(carAngle) * math.sin(goalAngle))
    dir = math.cos(carAngle) * math.sin(goalAngle) - math.sin(carAngle) * math.cos(goalAngle)
    return math.copysign(mag, dir)

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z # in radians

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
    plt.xlim(-2.4, 2.0)
    plt.ylim(-1.9, 1.6)
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

#Codigo necesario para que el autominy ejecute cada uno de los movimientos primitivos
def movimiento(mov):
    global vel
    global dir
    if(mov == 1):
        vel.value = 0.1
        dir.value = 0.0
    elif(mov == 2):
        vel.value = 0.1
        dir.value = 1.0
    elif(mov == 3):
        vel.value = 0.1
        dir.value = -1.0
    elif(mov == 4):
        vel.value = -0.1
        dir.value = 0.0
    elif(mov == 5):
        vel.value = -0.1
        dir.value = 1.0
    elif(mov == 6):
        vel.value = -0.1
        dir.value = -1.0
    else:
        vel.value = 0.0
        dir.value = 0.0



rospy.init_node("rrtnode") 
rate = rospy.Rate(1)
pubvel = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=1)
pubdir = rospy.Publisher("/actuators/steering_normalized",  NormalizedSteeringCommand, queue_size=1)
listener = tf.TransformListener()
vel = SpeedCommand()
dir = NormalizedSteeringCommand()

#Se obtiene la pocicion y orientacion del vehiculo
existe_angulo = 0
while(existe_angulo == 0):
    try:
        (trans,rot) = listener.lookupTransform('ar_marker_2', 'ar_marker_0', rospy.Time())
        carAngle = euler_from_quaternion (rot[0], rot[1], rot[2], rot[3]) 
        existe_angulo = 1
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        existe_angulo = 0

#Se obtiene la pocicion y orientacion de la meta
existe_angulo = 0
while(existe_angulo == 0):
    try:
        (trans_meta,rot_meta) = listener.lookupTransform('ar_marker_2', 'ar_marker_1', rospy.Time())
        metaAngle = euler_from_quaternion (rot[0], rot[1], rot[2], rot[3]) 
        existe_angulo = 1
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        existe_angulo = 0
 

#Movimientos primitivos reales
movs = [
    [0, 0, 0, 0], #Detenido
    [1, 0.5, 0.0, 0.0], #Hacia delante
    [2, 0.4, 0.24, 0.9], #Hacia delante e izquierda
    [3, 0.37, -0.28, -0.7], #Hacia delante y derecha
    [4, -0.5, 0.0, 0.0], #Hacia atrás
    [5, -0.41, 0.06, -0.85], #Hacia atrás e izquierda 
    [6, -0.46, -0.06, 0.70] #Hacia atrás y derecha
]
movs_cp = copy.deepcopy(movs)

# Crear un grafo vacío
grafo = nx.DiGraph()

# Agregar un nodo raíz (inicio)
agregar_nodo(grafo, 0, None, trans[0], trans[1], carAngle, 0)

# Meta
meta = [trans_meta[0], trans_meta[1], metaAngle]

#Variables para buscar el nodomas cercano al punto aleatorio
id_nuevo_nodo = 1
best_hoja = 0

#Variable usada para mostrar en el grafo que tipo de movimiento es cada nodo
texto_adicional_nodos = {0: '0'}

#Número máximo de veces que se ejecutara en algoritmo de RRT
for _ in range(400):
    #Reinicio de error y punto aleatorio
    best_error = 1000
    x_rnd = random.uniform(-2.4, 2.0)
    y_rnd = random.uniform(-1.9, 1.6)

    #Recorrer todo el arreglo buscando el nodo mas cercano al punto aleatorio
    for nodo_id, datos_nodo in grafo.nodes(data=True):
        error_actual = calcular_distancia(datos_nodo.get('x'), datos_nodo.get('y'), x_rnd, y_rnd)
        if(error_actual < best_error):
            best_nodo = nodo_id
            best_error = error_actual
            x_best = datos_nodo.get('x')
            y_best = datos_nodo.get('y')
            r_best = datos_nodo.get('r')

    #Se rotan los movimientos en base a la pocicion actual del vehiculo
    rotacion(r_best)
    
    #Se revisa si alguno de los 6 movimientos te hacerca a el punto aleatorio
    for mov_i in range(1, 7):
        err_prox = calcular_distancia(x_best+movs[mov_i][1], y_best+movs[mov_i][2], x_rnd, y_rnd)
        if(err_prox < best_error and -2.4 <= x_best+movs[mov_i][1] <= 2.0 and -1.9 <= y_best+movs[mov_i][2] <= 1.6):
            agregar_nodo(grafo, id_nuevo_nodo, best_nodo, x_best+movs[mov_i][1], y_best+movs[mov_i][2], r_best+movs[mov_i][3], mov_i)
            agregar_arista(grafo, best_nodo, id_nuevo_nodo)
            texto_adicional_nodos[id_nuevo_nodo] = mov_i 
            id_nuevo_nodo += 1
    
    #Se reinicia la orientacion de movimientos
    movs = copy.deepcopy(movs_cp)


#Encontrar nodo mas cercano a la meta
best_error = 1000
for nodo_id, datos_nodo in grafo.nodes(data=True):
        error_actual = calcular_distancia(datos_nodo.get('x'), datos_nodo.get('y'), meta[0], meta[1])*9 + AngDif(datos_nodo.get('r'), meta[2])*0.1
        if(error_actual < best_error):
            best_nodo = nodo_id
            best_error = error_actual

#Se dibujara el grafo para esto el inicio se colocara en color amarrillo miestras que el final en rojo 
#El manino recorrido para pasar del inicio al final se dibujara en color verde
nodo_meta = best_nodo
colores_nodos = {0: 'yellow'}
colores_nodos[best_nodo] = 'red'

#movs2do es un arreglo que almacenara todos los movimientos necesarios para hacer el recorrido
movs2do = []
movs2do.append(grafo.nodes[nodo_meta]['tipo'])
while(True):
    actual_nodo = grafo.nodes[best_nodo]['id_padre']
    if(grafo.nodes[actual_nodo]['id_padre'] == None):
        break
    movs2do.append(grafo.nodes[actual_nodo]['tipo'])
    colores_nodos[actual_nodo] = 'green'
    best_nodo = actual_nodo

#Se muestra informacion importanto como la posicion y orientacion final esperada del autominy 
print("El nodo mas cercano a la meta es: ", nodo_meta)
print(f"La posicion final de carrito es ({grafo.nodes[nodo_meta]['x']}, {grafo.nodes[nodo_meta]['y']})")
print(f"Con una rotacion de: {grafo.nodes[nodo_meta]['r']} radianes")

#Se muestra el numero de movimientos por hacer asi como la secuencia de estos mismos
Nmovs2do = len(movs2do)
print("Numero de movimientos por hacer: ", Nmovs2do)
movs2do = movs2do[::-1]
print("Secuencia de movimientos por hacer: ", movs2do)

# Dibujar grafo
dibujar_grafo(grafo, colores_nodos, texto_adicional_nodos)

#Como los movimientos se ejecutan en unm lapso de 4 segundos cada uno se inicializa el tiempo 
# y se establece que el movimiento inicial tarde un segundo mas devido a que tarda en inicial el movimiento del vehiculo
start_time = rospy.get_time()
mov_time = 5.0
mov_actual = 0

#Se recorre nuestro arreglo de movimientos ejecutando uno cada 4 segundos
print("Siguiente movimiento: ", movs2do[mov_actual])
while not rospy.is_shutdown():
    if(rospy.get_time() - start_time < mov_time):
        movimiento(movs2do[mov_actual])
    else:
        if(mov_actual < Nmovs2do-1):
            mov_time += 4.0
            mov_actual += 1
            print("Movimiento: ", movs2do[mov_actual])
        else:
            print("se acabo")
            movimiento(0)
    pubvel.publish(vel)
    pubdir.publish(dir)
    rospy.sleep(0.1)

rate.sleep()