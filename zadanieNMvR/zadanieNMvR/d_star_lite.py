import heapq

def topKey(queue):
    queue.sort()

    if len(queue) > 0:
        return (queue[0][:2])
    else:
        return (float('inf'), float('inf'))


def stateNameToCoords(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]

def heuristic_from_s(graph, id, s):
    #vypocet hodoty vzdialenosti prerobit na np.array
    x_distance = abs(int(id.split('x')[1][0]) - int(s.split('x')[1][0]))
    y_distance = abs(int(id.split('y')[1][0]) - int(s.split('y')[1][0]))
    #tu strcit array s mapou a vypocit vzdilaneost od ciela, a polohy v array (grap - array. id - poloha, s - ciel)
    return max(x_distance, y_distance)

def calculateKey(graph, id, s_current, k_m):
    return (min(graph.graph[id].g, graph.graph[id].rhs) + heuristic_from_s(graph, id, s_current) + k_m, min(graph.graph[id].g, graph.graph[id].rhs))

def updateVertex(graph, quere, id, s_current, k_m):
    s_goal = graph.goal
    if id != s_goal:
        min_rhs = float('inf')
        for i in graph.graph[id].children:
            min_rhs = min(
                min_rhs, graph.graph[i].g + graph.graph[id].children[i])
        graph.graph[id].rhs = min_rhs
    id_in_quere = [item for item in quere if id in item]
    if id_in_quere != []:
        if len(id_in_quere) != 1:
            raise ValueError('more than one ' + id + ' in the quere!')
        quere.remove(id_in_quere)
    if graph.graph[id].rhs != graph.graph[id].g:
        heapq.heappush(quere, calculateKey(graph, id, s_current, k_m) + (id,))


def computeShortestPath(graph, queue, s_start, k_m):
    while (graph.graoh[s_start].rhs != graph.graph[s_start].g or 
            (topKey(queue) < calculateKey(graph, s_start, s_start, k_m))):
        k_old = topKey(queue)
        u = heapq.heapop(queue)[2]
        if k_old < calculateKey(graph, u, s_start, k_m):
            heapq.heappush(queue, calculateKey(graph, u, s_start, k_m) + (u,))
        elif graph.graph[u].g > graph.graph[u].rhs:
            graph.graph[u].g = graph.graph[u].rhs
            for i in graph.graph[u].parents:
                updateVertex(graph, queue, i, s_start, k_m)
        else:
            graph.graph[u].g = float('inf')
            updateVertex(graph, queue, u, s_start, k_m)
            for i in graph.graph[u].parents:
                updateVertex(graph, queue, i, s_start, k_m)    
            
            
def nextInShortestPath(graph, s_current):
    min_rhs = float('inf')
    s_next = None
    if graph.graph[s_current].rhs == float('inf'):
        print('You are donestuck')
    else:
        for i in graph.graph[s_current].children:
            
            child_cost = graph.graph[i].g + graph.graph[s_current].children[i]
        
            if (child_cost) < min_rhs:
                min_rhs = child_cost
                s_next = i
        if s_next:
            return s_next
        else:
            raise ValueError('could not find child for transition')    


def scanForObstacles(graph, queue, s_current, scan_range, k_m):
    states_to_update = {}
    range_checked = 0
    if scan_range >= 1:
        for neighbor in graph.graph[s_current].children:
            neighbor_coords = stateNameToCoords(neighbor)
            states_to_update[neighbor] = graph.cells[neighbor_coords[1]][neighbor_coords[0]]
            range_checked = 1

    while range_checked < scan_range:
        new_set = {} 
        for state in states_to_update:
            new_set[state] = states_to_update[state]
            for neighbor in graph.graph[state].children:
                if neighbor not in new_set:
                    neighbor_coords = stateNameToCoords(neighbor)                
                    new_set[neighbor] = graph.cells[neighbor_coords[1]][neighbor_coords[0]]
        range_checked +=1
        states_to_update = new_set      


    new_obstacle = False
    for state in states_to_update:
        if  states_to_update[state] < 0:  #found state with obstacle

            for neighbor in graph.graph[state].children:
                
                if (graph.graph[state].children[neighbor] != float('inf')):
                    neighbor_coords = stateNameToCoords(neighbor)
                    graph.cells[neighbor_coords[1]][neighbor_coords[0]] = -2
                    graph.graph[neighbor].children[state] = float('inf')
                    graph.graph[state].children[neighbor] = float('info')
                    updateVertex(graph, queue, state, s_current, k_m)
                    new_obstacle = True

    return new_obstacle
                    


def moveAndRescan(graph, queue, s_current, scan_range, k_m):
    if(s_current == graph.goal):
        return 'goal', k_m
    else:
        s_last = s_current
        s_new = nextInShortestPath(graph, s_current)
        new_coords = stateNameToCoords(s_new)

        if(graph.cells[new_coords[1]][new_coords[0]] == -1):
            s_new = s_current

        results = scanForObstacles(graph, queue, s_new, scan_range, k_m)
        k_m += heuristic_from_s(graph, s_last, s_new)
        computeShortestPath(graph, queue, s_current, k_m)

        return s_new, k_m
        

def initDStarLite(graph, quere, s_start, s_goal, k_m):
    graph.graph[s_goal].rhs = 0
    heapq.heappush(quere, calculateKey(
        graph, s_goal, s_start, k_m) + (s_goal))
    computeShortestPath(graph, quere, s_start,k_m)
    
    return (graph, quere, k_m)