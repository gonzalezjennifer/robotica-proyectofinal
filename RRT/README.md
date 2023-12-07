## RRT IDEAL
Se crea un grafo y como nodo raíz se coloca la posición inicial del vehiculó como argumentos este nodo tiene el grafo, un id que debe ser único como es el nodo raíz debe de ser 0, el nodo padre que en este caso no tiene, la posición en x, y y su rotación y por ultimo el tipo de movimiento del 0 al 6 antes definidos en la matriz de movimientos. Como meta solo se requiere un arreglo con posición x, y, además de la orientación final deseada. <br><br>
![Creacion del nodo raiz](./img/2.JPG)

Debido al funcionamiento del RRT se genera un punto aleatorio al inicio de cada ciclo que es al punto al cual nos intentaremos acercar mediante los movimientos posibles del automóvil. <br><br>
![Generacion de puntos aleatorios](./img/3.JPG)

Se recorre todo el grafo generado hasta el momento buscando el nodo mas cercano al punto aleatorio anteriormente generado, esto mediante la función de calcular_distancia en donde se manda el punto x, y del nodo y el punto aleatorio al que se quiere acercarse, esto es lo que se conoce como el error de distancia a ese punto. Al final de este ciclo obtendremos el id del nodo mas cercano a este punto asi como otras propiedades de este como su punto x, y su rotación y el error de este nodo al punto aleatorio. <br><br>
![Recorrido del grafo](./img/4.JPG)

Se debe rotar la matriz de movimientos dependiendo de la posición de automóvil en el nodo más cercano al punto aleatorio obtenida en el ciclo anterior. <br><br>
![Rotacion de la matriz](./img/5.JPG)

Funcion que realiza la rotación: <br><br>
![Funcion de rotacion](./img/6.JPG)

Ecuaciones de rotacion: <br><br>
![Ecuaciones de rotacion](./img/7.jpg)

Este ciclo itera en los 6 movimientos posibles que puede hacer el automóvil en base al nodo mas cercano al punto aleatorio obtenido anteriormente. Se considera como best_error al error de este nodo y como err_prox al error que se obtendría al realizar el cada uno de los 6 movimientos, el condicional if verifica si el err_prox es menor al best_error (es decir que si el posible movimiento acerca más a el punto aleatorio que se desea alcanzar). Aquellos movimientos que se alejen del punto aleatorio y que se salgan de los limites del plano al realizarlos no son dibujados en el grafo por lo que quedan descartados. <br><br>
![Filtracion de movimientos](./img/8.JPG)

Se restaura la matriz de movimientos a su estado original para que en la siguiente iteración se aplique la rotación necesaria y sea así rotación relativa a la orientación actual del automóvil y no una rotación acumulada <br><br>
![Copia de la matriz original](./img/9.JPG)

Una vez terminada la exploración del algoritmo se recorre todo el grafo, pero esta vez buscando el nodo mas cercano a la meta y no a un punto aleatorio para esto también se toma en cuenta la orientación final que se desea obtener, pero dándole un peso de 10% para orientación mientras que a la distancia un peso de 90%. Al final de esto obtendremos el nodo mas cercano a esta meta y que además tenga una orientación dentro de un rango de los que se desea obtener.<br><br>
![Ciclo para obtener los nodos mas cercanos a la meta](./img/10.JPG)

Se recorre el grafo en sentido contrario empezando con el nodo mas cercano a la meta y terminando en el nodo raíz esto gracias a que cada nodo tiene un padre excepto el nodo raíz que sería nuestra posición y orientación inicial. A la vez que se realiza esto se guarda la serie de movimientos que se debe seguir para llegar del nodo raíz (inicio) al nodo más cercano a la meta. <br><br>
![Guardado de moviemientos a seguir](./img/11.JPG)

Correr con ```python proyecto.py``` obteniendo un resultado como el siguiente. <br><br>
![Planeacion de movimientos](./img/1.JPG)

> **Nota:** Este codigo actualmente no funciona con el autominy.