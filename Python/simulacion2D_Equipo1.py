import agentpy as ap
import matplotlib.pyplot as plt
import IPython
from collections import deque
import random

def draw_tile(graph, id, style):
    # Inicializa la variable 'r' con un espacio vacío representado por " . "
    # que será el valor predeterminado si no se cumple ninguna de las condiciones posteriores.
    r = " . "
    
    # Si el diccionario 'style' contiene una clave 'number' y el 'id' está en ese diccionario,
    # establece 'r' en un número formateado con dos dígitos.
    if 'number' in style and id in style['number']: 
        r = " %-2d" % style['number'][id]
    
    # Si el diccionario 'style' contiene una clave 'point_to' y 'id' está presente en 'point_to',
    # entonces verifica la dirección hacia la que apunta, y establece 'r' en una flecha 
    # indicando la dirección.
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id  # Coordenadas del punto actual.
        (x2, y2) = style['point_to'][id]  # Coordenadas del punto hacia el que apunta.

        # Define la representación gráfica de la dirección:
        if x2 == x1 + 1: r = " > "  # Moverse hacia la derecha.
        if x2 == x1 - 1: r = " < "  # Moverse hacia la izquierda.
        if y2 == y1 + 1: r = " v "  # Moverse hacia abajo.
        if y2 == y1 - 1: r = " ^ "  # Moverse hacia arriba.
    
    # Si 'style' tiene una clave 'path' y el 'id' está en 'path',
    # establece 'r' en " @ " para representar que es parte del camino.
    if 'path' in style and id in style['path']:   
        r = " @ "
    
    # Si 'id' es el punto de inicio (clave 'start' en 'style'),
    # establece 'r' en " A " para marcarlo como el punto de inicio.
    if 'start' in style and id == style['start']: 
        r = " A "
    
    # Si 'id' es el punto final o meta (clave 'goal' en 'style'),
    # establece 'r' en " G " para marcarlo como el objetivo.
    if 'goal' in style and id == style['goal']:   
        r = " G "
    
    # Si el 'id' está en la lista de paredes del grafo, establece 'r' en "###"
    # para indicar que es una pared.
    if id in graph.walls: 
        r = "###"
    
    # Finalmente, devuelve el valor de 'r', que representa gráficamente el tile (casilla) en la cuadrícula.
    return r

def draw_grid(graph, **style):
    # Imprime una línea superior para el borde del grid, multiplicando "___" por el ancho del grafo.
    # Esto crea una separación visual al inicio del grid.
    print("___" * graph.width)
    
    # Itera sobre las filas del grid según la altura del grafo.
    for y in range(graph.height):
        # Dentro de cada fila, itera sobre las columnas del grid según el ancho del grafo.
        for x in range(graph.width):
            # Llama a la función 'draw_tile' para obtener la representación de la casilla en (x, y)
            # y la imprime sin saltar a la siguiente línea (por el argumento `end=""`).
            print("%s" % draw_tile(graph, (x, y), style), end="")
        
        # Al terminar de imprimir una fila completa, imprime un salto de línea para pasar a la siguiente fila.
        print()
    
    # Imprime una línea inferior para el borde del grid, similar a la superior.
    print("~~~" * graph.width)

# Importación de módulos necesarios para mejorar la legibilidad y definición de tipos.
from typing import Protocol, Iterator, Tuple, TypeVar, Optional, List, Dict

# Importación de funcionalidades de versiones futuras (útil para habilitar anotaciones de tipos más avanzadas).
from __future__ import annotations

# Definición de una variable de tipo genérico 'T', que puede representar cualquier tipo en tiempo de ejecución.
T = TypeVar('T')

# Definición de un tipo genérico 'Location', que también puede representar cualquier tipo, pero típicamente será usado para definir ubicaciones en un grid o grafo.
Location = TypeVar('Location')

# Definición de una clase llamada 'Graph' que sigue el protocolo de 'Protocol'.
# Un 'Protocol' en Python define la estructura esperada de clases que implementen este protocolo, similar a una interfaz.
class Graph(Protocol):
    # Método abstracto que se espera que las clases que hereden este protocolo implementen.
    # Este método devuelve una lista de ubicaciones (vecinos) basadas en una ubicación dada 'id'.
    def neighbors(self, id: Location) -> list[Location]: pass

# Definición de un tipo alias llamado 'GridLocation', que será representado como una tupla de dos enteros (x, y).
# Esto se usa para facilitar el manejo de ubicaciones en un grid bidimensional (por ejemplo, en un mapa o grafo).
GridLocation = Tuple[int, int]

# Clase que representa un grid (cuadrícula) de tamaño 'width' x 'height'.
class SquareGrid:
    # Constructor que inicializa la cuadrícula con el ancho y alto dados.
    def __init__(self, width: int, height: int):
        self.width = width  # Ancho de la cuadrícula.
        self.height = height  # Altura de la cuadrícula.
        self.walls: List[GridLocation] = []  # Lista que contiene las ubicaciones de las paredes u obstáculos.

    # Método que verifica si una posición 'id' está dentro de los límites del grid.
    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id  # Desempaquetamos la tupla (x, y) que representa la posición.
        # Retorna True si la posición está dentro de los límites del grid (0 <= x < width y 0 <= y < height).
        return 0 <= x < self.width and 0 <= y < self.height

    # Método que verifica si una posición 'id' es transitable (no es una pared).
    def passable(self, id: GridLocation) -> bool:
        # Retorna True si 'id' no está en la lista de paredes, es decir, si es transitable.
        return id not in self.walls

    # Método que devuelve un iterador de las posiciones vecinas a una posición 'id'.
    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id  # Desempaquetamos la tupla (x, y) que representa la posición.
        # Lista de vecinos: derecha, izquierda, abajo y arriba (Este, Oeste, Norte y Sur).
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)]
        
        # Invertimos el orden de los vecinos en filas alternas para diversificar el orden de búsqueda.
        if (x + y) % 2 == 0: neighbors.reverse()  # Inversión: Sur, Norte, Oeste, Este.

        # Filtramos los vecinos para que solo devuelva los que están dentro de los límites del grid.
        results = filter(self.in_bounds, neighbors)
        # Filtramos los vecinos para que solo devuelva los que son transitables (no paredes).
        results = filter(self.passable, results)

        # Devolvemos un iterador de las posiciones vecinas válidas.
        return results

# Clase que extiende la interfaz Graph, representando un grafo ponderado (WeightedGraph).
class WeightedGraph(Graph):
    # Método abstracto que define el costo de moverse de un nodo 'from_id' a otro nodo 'to_id'.
    # Debe ser implementado por las subclases para definir cómo calcular dicho costo.
    def cost(self, from_id: Location, to_id: Location) -> float:
        pass  # Placeholder para que las subclases implementen la lógica de cálculo de costos.

# Clase que extiende SquareGrid para incluir pesos en las ubicaciones del grid.
class GridWithWeights(SquareGrid):
    
    # Constructor que inicializa el grid con ancho, alto y un diccionario de pesos por ubicación.
    def __init__(self, width: int, height: int):
        # Llama al constructor de la clase padre SquareGrid para inicializar el tamaño del grid.
        super().__init__(width, height)
        # Diccionario que almacena los pesos para cada ubicación en el grid.
        # Si una ubicación no tiene un peso especificado, se asume que tiene un peso predeterminado.
        self.weights: dict[GridLocation, float] = {}

    # Método que calcula el costo de moverse de un nodo (from_node) a otro (to_node).
    # Si el nodo de destino (to_node) tiene un peso asignado, se devuelve ese valor.
    # De lo contrario, se devuelve un valor por defecto de 1.
    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        # Se obtiene el peso del nodo destino (to_node) del diccionario de pesos.
        # Si no se encuentra el nodo en el diccionario, el valor por defecto es 1.
        return self.weights.get(to_node, 1)

import heapq  # Se usa para la implementación de la cola de prioridad.

# Clase PriorityQueue que implementa una cola de prioridad usando un heap.
class PriorityQueue:
    
    # Constructor que inicializa la lista de elementos como una lista vacía.
    # Cada elemento es una tupla que contiene una prioridad y un ítem.
    def __init__(self):
        self.elements: list[tuple[float, T]] = []

    # Método que comprueba si la cola de prioridad está vacía.
    # Devuelve True si no hay elementos, de lo contrario devuelve False.
    def empty(self) -> bool:
        return not self.elements

    # Método para insertar un nuevo ítem en la cola de prioridad con su respectiva prioridad.
    # heapq.heappush se usa para mantener la lista ordenada automáticamente por prioridad.
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))

    # Método que extrae el ítem con la mayor prioridad (la más baja numéricamente) de la cola.
    # heapq.heappop devuelve el ítem con la prioridad más baja y se obtiene solo el ítem, no la prioridad.
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

# Función heurística que calcula la distancia aproximada entre dos ubicaciones en una cuadrícula.
# En este caso, usa la distancia de Manhattan, que es la suma de las diferencias absolutas
# entre las coordenadas x e y de los dos puntos.
def heuristic(a: GridLocation, b: GridLocation) -> float:
    # Desempaqueta las coordenadas (x1, y1) de la ubicación 'a' y (x2, y2) de la ubicación 'b'.
    (x1, y1) = a
    (x2, y2) = b
    
    # Calcula y devuelve la distancia de Manhattan entre los dos puntos.
    # La distancia de Manhattan es útil en cuadrículas cuando solo se permiten movimientos
    # en líneas rectas (horizontales o verticales).
    return abs(x1 - x2) + abs(y1 - y2)

# Implementación del algoritmo de búsqueda A* (A-star) en un grafo ponderado.
# Este algoritmo encuentra el camino más corto desde el nodo inicial 'start' hasta el nodo 'goal'.
def a_star_search(graph: WeightedGraph, start: Location, goal: Location):
    # Inicializa una cola de prioridad, donde se almacenan los nodos por explorar junto con su prioridad.
    frontier = PriorityQueue()
    frontier.put(start, 0)  # Coloca el nodo inicial en la cola con prioridad 0.

    # Diccionario que almacena el nodo previo desde el cual se llegó a cada nodo.
    came_from: dict[Location, Optional[Location]] = {}
    
    # Diccionario que almacena el costo acumulado para llegar a cada nodo desde el nodo inicial.
    cost_so_far: dict[Location, float] = {}
    
    # Inicializa el punto de partida: no tiene un nodo previo (None) y su costo acumulado es 0.
    came_from[start] = None
    cost_so_far[start] = 0

    # Mientras haya nodos por explorar en la cola de prioridad.
    while not frontier.empty():
        # Extrae el nodo con la prioridad más alta (menor costo estimado).
        current: Location = frontier.get()

        # Si se ha llegado al nodo objetivo, se termina la búsqueda.
        if current == goal:
            break

        # Recorre los vecinos del nodo actual.
        for next in graph.neighbors(current):
            # Calcula el nuevo costo para llegar al nodo vecino desde el nodo actual.
            new_cost = cost_so_far[current] + graph.cost(current, next)

            # Si el nodo vecino no ha sido explorado o se encuentra un camino más barato hacia él.
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                # Actualiza el costo acumulado para el nodo vecino.
                cost_so_far[next] = new_cost
                
                # Calcula la prioridad del nodo vecino, sumando el nuevo costo y la heurística hacia el objetivo.
                priority = new_cost + heuristic(next, goal)
                
                # Añade el nodo vecino a la cola de prioridad con su nueva prioridad.
                frontier.put(next, priority)
                
                # Registra que se llegó al nodo vecino desde el nodo actual.
                came_from[next] = current

    # Devuelve el diccionario 'came_from' con el historial del camino y el 'cost_so_far' con los costos acumulados.
    return came_from, cost_so_far

def reconstruct_path(came_from: dict[Location, Location],
                     start: Location, goal: Location) -> list[Location]:
    # Inicializa la variable 'current' con el objetivo (goal)
    current: Location = goal
    
    # Inicializa una lista vacía para almacenar el camino
    path: list[Location] = []
    
    # Si el objetivo no está en el diccionario 'came_from', no se encontró un camino
    if goal not in came_from:
        return []
    
    # Recorre el camino desde el objetivo hasta el inicio
    while current != start:
        path.append(current)  # Añade el nodo actual al camino
        current = came_from[current]  # Mueve al nodo anterior en el camino
    
    # Opcional: Añade el nodo de inicio al camino (si es necesario)
    path.append(start)
    
    # Opcional: Invierte el camino para que comience desde el inicio
    path.reverse()
    
    return path  # Devuelve el camino reconstruido

# Crear el grid para A* con los valores de los agentes
def create_a_star_grid(model):
    # Obtener el tamaño del grid del modelo
    size = model.p.size
    
    # Inicializa una matriz de costos de tamaño 'size x size' con ceros
    grid_costs = [[0] * size for _ in range(size)]

    # Extrae los valores de los agentes del grid
    attr_grid = model.campo.attr_grid('condition')

    # Recorre cada celda del grid
    for y in range(size):
        for x in range(size):
            # Obtiene el valor de la celda en 'attr_grid'
            value = attr_grid[y][x]
            
            # Asigna un costo a la celda basado en su valor
            if value == 0:  # Si el valor es 0 (tractor)
                grid_costs[y][x] = 300  # Asigna un costo alto (300)
            elif value == 1:  # Si el valor es 1 (unharvested)
                grid_costs[y][x] = 100  # Asigna un costo moderado (100)
            elif value == 2:  # Si el valor es 2 (harvested)
                grid_costs[y][x] = 5  # Asigna un costo bajo (5)
            elif value == 3:  # Si el valor es 3 (obstacle)
                grid_costs[y][x] = 10000  # Asigna un costo muy alto (10000)
            elif value == 4:  # Si el valor es 4 (warehouse)
                grid_costs[y][x] = 25  # Asigna un costo moderado (25)
            elif value == 5:  # Si el valor es 5 (harvester)
                grid_costs[y][x] = 300  # Asigna un costo alto (300)
            else:
                # Asigna un costo por defecto para valores no especificados
                grid_costs[y][x] = 5

    # Debug: imprime los costos del grid (comentado por defecto)
    #print("grid_costs:")
    #for row in grid_costs:
    #    print(row)

    # Regresa el grid de costos que se usará para el A*
    return grid_costs

# Crea el weighted grid
def create_weighted_grid(grid_costs):
    # Obtiene el ancho y alto del grid a partir de 'grid_costs'
    width = len(grid_costs[0])
    height = len(grid_costs)

    # Crea un grid para el A* (vacío) con dimensiones específicas
    weighted_grid = GridWithWeights(width, height)
    
    # Inicializa la lista de paredes como vacía
    weighted_grid.walls = []

    # Recorre cada celda del grid de costos
    for y in range(height):
        for x in range(width):
            # Obtiene el costo de la celda actual
            cost = grid_costs[y][x]
            
            # Si el costo no es 1000 (no es una celda de obstáculo)
            if cost != 1000:
                # Añade el costo al grid con pesos
                weighted_grid.weights[(y, x)] = cost
            else:
                # Añade la celda a la lista de paredes si el costo es 1000
                weighted_grid.walls.append((y, x))

    # Debug: imprime los detalles del grid con pesos (comentado por defecto)
    #print("weighted_grid.weights:")
    #for key, value in weighted_grid.weights.items():
    #    print(f"{key}: {value}")

    #print("weighted_grid.walls:")
    #print(weighted_grid.walls)

    # Regresa el grid con pesos
    return weighted_grid

class Harvest(ap.Agent):
    def setup(self):
        self.condition = 1  # 0: Tractor, 1: Unharvested, 2: harvested

    def collect(self):
        if self.condition == 1:
            self.condition = 2  # Marca la cosecha como recolectada
            model.crops_left -= 1
            model.crops_harvested += 1
            return 1  # Retorna 1 para indicar que una cosecha fue recolectada
        return 0  # Retorna 0 si ya estaba recolectada

class Obstacle(ap.Agent):
    def setup(self):
        self.condition = 3  # 3: Obstáculo

class Warehouse(ap.Agent):
    def setup(self):
        self.condition = 4 # 4 : Warehouse

class Harvester(ap.Agent):
    def setup(self):
        self.condition = 5 # 5 : Harvester
        self.campo = self.model.campo
        self.path = deque()
        self.to_truck = True

    def A_Star(self, start: GridLocation, goal) -> Optional[GridLocation]:
    # Crea el grid de costos para el algoritmo A* usando el modelo
        grid_costs = create_a_star_grid(self.model)
    
    # Crea el grid con pesos a partir del grid de costos
        weighted_grid = create_weighted_grid(grid_costs)

    # Ejecuta el algoritmo A* para encontrar el camino desde el inicio al objetivo
    # 'came_from' contiene los nodos predecesores en el camino encontrado
        came_from, _ = a_star_search(weighted_grid, start, goal)
    
    # Reconstruye el camino a partir de los nodos predecesores
        path = reconstruct_path(came_from, start, goal)

    # DEBUG !!
    # Imprime la posición inicial y final, y el camino encontrado
    #print("Step", self.model.t)
    #print("Start pos", start)
    #print("Goal", goal)
    #print("Path", path)
    # Dibuja el grid con el camino encontrado (comentado por defecto)
    #draw_grid(weighted_grid, path=path)

    # Regresa el camino encontrado por el algoritmo A*
        return path

    def find_closest_tractor(self):
        # Inicializa la distancia mínima como infinito y el tractor más cercano como None
        min_distance = float('inf')
        closest_tractor = None
        # Recorre todos los tractores en el modelo
        for tractor in model.tractores:
            # Verifica que el tractor no esté en movimiento y no tenga un cosechador asignado
            if not tractor.isMoving and not tractor.harvesterAssigned:
                # Calcula la distancia entre la posición actual y la del tractor
                distance = self.calculate_distance(model.campo.positions[self], model.campo.positions[tractor])
                # Si la distancia calculada es menor que la distancia mínima encontrada hasta ahora
                if distance < min_distance:
                    # Actualiza la distancia mínima y el tractor más cercano
                    min_distance = distance
                    closest_tractor = tractor
        # Si se encontró un tractor cercano
        if closest_tractor is not None:
            # Marca el tractor como asignado a un cosechador
            closest_tractor.harvesterAssigned = True
            # Regresa la posición del tractor más cercano si se encontró alguno, de lo contrario, regresa None
        return model.campo.positions[closest_tractor] if closest_tractor else None

    def find_closest_warehouse(self):
        min_distance = float('inf')
        closest_warehouse = None
        for warehouse in model.warehouses:
            distance = self.calculate_distance(model.campo.positions[self], model.campo.positions[warehouse])
            if distance < min_distance:
                min_distance = distance
                closest_warehouse = warehouse
        return model.campo.positions[closest_warehouse] if closest_warehouse else None

    def calculate_distance(self, pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def is_valid_position(self, pos):
      # Verifica si la posición está dentro de los límites del campo.
      # Comprueba si la coordenada x (pos[0]) y la coordenada y (pos[1]) están dentro del rango permitido.
      if 0 <= pos[0] < self.model.p.size and 0 <= pos[1] < self.model.p.size:

          # Obtiene todos los agentes en la posición especificada.
          agents_in_pos = list(self.campo.agents[pos])

          # Comprueba si no hay ningún obstáculo en la posición.
          # Recorre todos los agentes en la posición y verifica si alguno es una instancia de Obstacle.
          if not any(isinstance(agent, Obstacle) or isinstance(agent, Tractor) for agent in agents_in_pos):
              # Si no hay obstáculos, la posición es válida.
              return True

      # Si la posición está fuera de los límites o hay un obstáculo, no es válida.
      return False

    def move(self):
        if self.path:
            next_pos = self.path[0] #Get next pos in path
            agents_in_pos = list(self.campo.agents[next_pos]) #get agents in next pos (can be overlapping)
            if not any(isinstance(agent, Obstacle) for agent in agents_in_pos): #if they're not obstacles
                for agent in agents_in_pos:
                    if isinstance(agent, Tractor): # if next pos is a tractor
                        if self.to_truck: # and was originally going to one
                            if not agent.isMoving:
                              agent.capacity = self.p['capacity'] # resets tractor capacity
                              agent.isMoving = True
                              agent.harvesterAssigned = False
                              self.to_truck = False # and is now moving to warehouse
                              # if it wasn't going to a tractor and found one, it needs to recalculate its route to
                              # not clash; if it was going to one, then it has fulfilled its objective
                              self.path.clear()
                        break
                    elif isinstance(agent, Warehouse): # if next pos is a warehouse
                        # then it can now go to a truck. If it was going to a truck originally, it doesn't
                        # matter
                        self.to_truck = True
            # if the path didn't get cleared
            if self.path:
                self.path.popleft()
                self.campo.move_to(self, next_pos)
        else:

            current_pos = self.campo.positions[self]

            if self.to_truck:
                tractor = self.find_closest_tractor()
                if tractor is not None:
                    self.path = deque(self.A_Star(current_pos, tractor))
            else:
                warehouse = self.find_closest_warehouse()
                self.path = deque(self.A_Star(current_pos, warehouse))

class Tractor(ap.Agent):
    def setup(self):
        self.condition = 0
        self.capacity = self.p['capacity']  # Initialize with the given capacity parameter
        self.initial_capacity = self.capacity  # Store the initial capacity
        self.campo = self.model.campo
        self.warehouse_path = deque()
        self.isMoving = True
        self.harvesterAssigned = False

    def move_and_collect(self):
        # Obtiene la posición actual del tractor
        current_pos = self.campo.positions[self]
        if self.capacity > 0:  # Si el tractor aún tiene capacidad
            # Encuentra la cosecha más cercana, evitando obstáculos
            target_pos = self.find_closest_harvest(current_pos)
            path = self.bfs(current_pos, target_pos)
            if len(path) > 1:
                # Mueve el tractor a la siguiente posición en el camino
                next_pos = path[1]
            else:
                # Si ya estamos en el objetivo, no se mueve
                next_pos = current_pos

            # Verifica si la siguiente posición es válida
            if self.is_valid_position(next_pos):
                # Obtiene todos los agentes en la próxima posición
                agents_in_pos = list(self.campo.agents[next_pos])

                # Itera sobre los agentes en la próxima posición
                for agent in agents_in_pos:
                    # Si encuentra una cosecha y el tractor aún tiene capacidad
                    if isinstance(agent, Harvest) and self.capacity > 0:
                        # Recolecta la cosecha y reduce la capacidad del tractor
                        self.capacity -= agent.collect()
                        if self.capacity == 0:
                            self.isMoving = False
                        break

                # Mueve el tractor a la siguiente posición
                self.campo.move_to(self, next_pos)

    def is_valid_position(self, pos):
      # Verifica si la posición está dentro de los límites del campo.
      # Comprueba si la coordenada x (pos[0]) y la coordenada y (pos[1]) están dentro del rango permitido.
      if 0 <= pos[0] < self.model.p.size and 0 <= pos[1] < self.model.p.size:

          # Obtiene todos los agentes en la posición especificada.
          agents_in_pos = list(self.campo.agents[pos])

          # Comprueba si no hay ningún obstáculo en la posición.
          # Recorre todos los agentes en la posición y verifica si alguno es una instancia de Obstacle.
          if not any(isinstance(agent, Obstacle) or isinstance(agent, Tractor) or isinstance(agent, Warehouse) or isinstance(agent, Harvester) for agent in agents_in_pos):
              # Si no hay obstáculos, la posición es válida.
              return True

      # Si la posición está fuera de los límites o hay un obstáculo, no es válida.
      return False


    def find_closest_harvest(self, start):
      # Inicia una cola (queue) con la posición inicial (start)
      queue = deque([start])

      # Crea un conjunto (set) para rastrear las posiciones visitadas
      visited = set()

      # Marca la posición inicial como visitada
      visited.add(start)

      # Comienza a recorrer la cola mientras no esté vacía
      while queue:
          # Extrae la primera posición de la cola
          pos = queue.popleft()

          # Verifica si la posición actual contiene algún agente
          if pos in self.campo.positions.values():
              # Obtiene todos los agentes en la posición actual
              agents_in_pos = list(self.campo.agents[pos])

              # Itera sobre los agentes en la posición
              for agent in agents_in_pos:
                  # Si el agente es una cosecha (Harvest) y su condición es 1 (disponible para recolección)
                  if isinstance(agent, Harvest) and agent.condition == 1:
                      # Devuelve la posición de esta cosecha
                      return pos

          # Si no se encuentra una cosecha en la posición actual, obtiene las posiciones vecinas
          neighbors = self.get_neighbors(pos)

          # Itera sobre las posiciones vecinas
          for neighbor in neighbors:
              # Si la posición vecina no ha sido visitada, la marca como visitada
              if neighbor not in visited:
                  visited.add(neighbor)
                  # Añade la posición vecina a la cola para procesarla después
                  queue.append(neighbor)

      # Si no se encuentra ninguna cosecha válida, devuelve None
      return None


    def bfs(self, start, goal):
      # Inicia una cola con un solo camino que comienza en la posición inicial (start)
      queue = deque([[start]])

      # Crea un conjunto para rastrear las posiciones visitadas
      visited = set()

      # Marca la posición inicial como visitada
      visited.add(start)

      # Comienza a recorrer la cola mientras no esté vacía
      while queue:
          # Extrae el primer camino de la cola
          path = queue.popleft()

          # Obtiene la última posición en el camino actual
          current = path[-1]

          # Si la posición actual es el objetivo, devuelve el camino que llevó hasta allí
          if current == goal:
              return path

          # Recorre todas las posiciones vecinas de la posición actual
          for neighbor in self.get_neighbors(current):
              # Si la posición vecina no ha sido visitada, procede
              if neighbor not in visited:
                  # Marca la posición vecina como visitada
                  visited.add(neighbor)

                  # Crea un nuevo camino que incluye esta posición vecina
                  new_path = list(path)
                  new_path.append(neighbor)

                  # Añade el nuevo camino a la cola para explorarlo más tarde
                  queue.append(new_path)

      # Si no se encuentra un camino al objetivo, devuelve un camino que solo contiene la posición inicial
      return [start]


    def get_neighbors(self, pos):
      # Crea una lista vacía para almacenar los vecinos válidos
      neighbors = []

      # Define las direcciones posibles de movimiento: derecha, izquierda, abajo, arriba
      directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

      # Recorre cada dirección posible
      for direction in directions:
          # Calcula la nueva posición vecina sumando la dirección a la posición actual
          neighbor = (pos[0] + direction[0], pos[1] + direction[1])

          # Verifica si la nueva posición está dentro de los límites del campo
          if 0 <= neighbor[0] < self.model.p.size and 0 <= neighbor[1] < self.model.p.size:
              # Obtiene todos los agentes en la nueva posición vecina
              agents_in_pos = list(self.campo.agents[neighbor])

              # Comprueba si la nueva posición no contiene un obstáculo
              if not any(isinstance(agent, Obstacle) or isinstance(agent, Warehouse) for agent in agents_in_pos):
                  # Si no hay obstáculo, añade la posición vecina a la lista de vecinos válidos
                  neighbors.append(neighbor)

      # Devuelve la lista de vecinos válidos
      return neighbors

class HarvestModel(ap.Model):
    def setup(self):
        # Initialize the grid
        self.campo = ap.Grid(self, [self.p.size] * 2, track_empty=True)
        self.agents = ap.AgentList(self)

        # Create and place warehouses
        self.warehouses = ap.AgentList(self, self.p['warehouses'], Warehouse)
        self.campo.add_agents(self.warehouses, random=True, empty=True)

        # Create and place tractors
        self.tractores = ap.AgentList(self, self.p['tractores'], Tractor)
        self.campo.add_agents(self.tractores, random=True, empty=True)

        # Create harvesters list
        self.harvesters = ap.AgentList(self, self.p['tractores'], Harvester)  # Assuming number of harvesters equals number of tractors

        # Assign positions to harvesters
        self.assign_harvester_positions()

        # Place harvesters
        self.campo.add_agents(self.harvesters, positions=self.harvester_positions)

        # Calculate the number of available cells for obstacles and crops
        total_cells = self.p.size ** 2
        available_cells = total_cells - len(self.tractores) - len(self.warehouses) - len(self.harvesters)

        # Create obstacles and crops if there are available cells
        max_obstacles = min(self.p['obstacles'], available_cells)
        max_crops = min(int(self.p['harvest_density'] * available_cells), available_cells - max_obstacles)

        if max_obstacles > 0:
            obstacles = ap.AgentList(self, max_obstacles, Obstacle)
            self.campo.add_agents(obstacles, random=True, empty=True)

        if max_crops > 0:
            crops = ap.AgentList(self, max_crops, Harvest)
            self.campo.add_agents(crops, random=True, empty=True)

        # Initialize crop counters
        self.crops_left = max_crops
        self.crops_harvested = 0
        self.crops_collected = []

    def get_adjacent_positions(self, position):
        x, y = position
        adj_positions = [
            (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)
        ]
        return [p for p in adj_positions if 0 <= p[0] < self.p.size and 0 <= p[1] < self.p.size]

    def assign_harvester_positions(self):
        self.harvester_positions = []
        for tractor in self.tractores:
            tractor_pos = self.campo.positions[tractor]
            adjacent_positions = self.get_adjacent_positions(tractor_pos)
            chosen_position = random.choice(adjacent_positions)
            self.harvester_positions.append(chosen_position)

        if len(self.harvester_positions) < len(self.harvesters):
            print("Not all harvesters could be assigned valid positions.")
            # Optionally handle the case where not all harvesters could be placed

    def step(self):
        if self.crops_left > 0:
            for tractor in self.tractores:
                if tractor.isMoving:
                    tractor.move_and_collect()

            for harvester in self.harvesters:
                harvester.move()
        else:
            self.stop()

def animation_plot(model, ax):
    attr_grid = model.campo.attr_grid('condition')

    # Definir los colores para los diferentes agentes
    color_dict = {
        0: '#008000',  # Tractor - Dark Green
        1: '#fff789',  # Unharvested - Yellow
        2: '#9B870C',  # Harvested - Gold
        3: '#1E2E38',  # Obstacle - Black
        4: '#903b1c',  # Warehouse - Red
        5: '#795695',  # Harvester - purple
        None: '#d5e5d5'  # Empty Space - Light Gray
    }

    # Plot el grid
    ap.gridplot(attr_grid, ax=ax, color_dict=color_dict, convert=True)

    # Superposición de las capacidades del tractor
    for tractor in model.tractores:
        pos = model.campo.positions[tractor]
        ax.text(pos[1], pos[0], f'Cap: {tractor.capacity}',
                ha='center', va='center', color='white', fontsize=8,
                bbox=dict(facecolor='black', alpha=0.5, edgecolor='none'))

    for harvester in model.harvesters:
        pos = model.campo.positions[harvester]
        ax.text(pos[1], pos[0], f'Harvester',
                ha='center', va='center', color='white', fontsize=8,
                bbox=dict(facecolor='black', alpha=0.5, edgecolor='none'))

    # Agregar título con el paso de tiempo actual y los cultivos restantes
    ax.set_title(f"Simulation of a Harvest\n"
                 f"Time-step: {model.t}, Crops left: "
                 f"{model.crops_left}")

parameters = {
    'steps': 400,
    'size': 25,
    'harvest_density': 1,
    'obstacles': 25,
    'tractores' : 3, # el numero de harvesters es igual
    'warehouses' : 3,
    'capacity' : 50,
}

# Configurar la animación
fig, ax = plt.subplots(figsize=(8, 8))
model = HarvestModel(parameters)
animation = ap.animate(model, fig, ax, animation_plot)
IPython.display.HTML(animation.to_jshtml(fps=10))