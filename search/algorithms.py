
import heapq

class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node. 
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._h = 0
        self._cost = 0
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def get_h(self):
        """
        Returns the h-value of the state
        """
        return self._h
    
    def get_cost(self):
        """
        Returns the cost of the state (g for Dijkstra's and f for A*)
        """
        return self._cost
    
    def set_g(self, cost):
        """
        Sets the g-value of the state
        """
        self._g = cost
    
    def set_h(self, h):
        """
        Sets the h-value of the state
        """
        self._h = h
    
    def set_cost(self, cost):
        """
        Sets the cost of a state (g for Dijkstra's and f for A*)
        """
        self._cost = cost

class Search:
    """
    Interface for a search algorithm. It contains an OPEN list and a CLOSED list.

    The OPEN list is implemented with a heap, which can be done with the library heapq
    (https://docs.python.org/3/library/heapq.html).    
    
    The CLOSED list is implemented as a dictionary where the state hash value is used as key.
    """
    def __init__(self, gridded_map):
        self.map = gridded_map
        self.OPEN = []
        self.CLOSED = {}
    
    def search(self, start, goal):
        """
        Search method that needs to be implemented (either Dijkstra or A*).
        """
        raise NotImplementedError()
            
class Dijkstra(Search):
            
    def search(self, start, goal):
        """
        Disjkstra's Algorithm: receives a start state and a goal state as input. It returns the
        cost of a path between start and goal and the number of nodes expanded.

        If a solution isn't found, it returns -1 for the cost.
        """
        self.OPEN = []
        self.CLOSED = {}
        
        heapq.heappush(self.OPEN, start)
        self.CLOSED[start.state_hash()] = start.get_cost()
        node_expanded = 0
        
        while self.OPEN:
            n = heapq.heappop(self.OPEN)
            node_expanded += 1
            
            if n == goal:   # goal found
                return self.CLOSED[n.state_hash()], node_expanded
            
            for childn in self.map.successors(n):
                # n’ is not expanded before                
                if childn.state_hash() not in self.CLOSED:
                    childn.set_cost(childn.get_g())
                    heapq.heappush(self.OPEN, childn)
                    self.CLOSED[childn.state_hash()] = childn.get_cost()
    
                # childn is expanded before, but a cheaper path is found to childn
                elif childn.state_hash() in self.CLOSED and childn.get_g() < self.CLOSED[childn.state_hash()]:                    
                    childn.set_cost(childn.get_g())
                    heapq.heapify(self.OPEN)
                    self.CLOSED[childn.state_hash()] = childn.get_g()                    
        
        return -1, 0
    
class AStar(Search):
    
    def h_value(self, state, goal):
        x, y = abs(state.get_x() - goal.get_x()), abs(state.get_y() - goal.get_y())
        return max(x,y) + 0.5 * min(x,y)
            
            
    def search(self, start, goal):
        """
        A* Algorithm: receives a start state and a goal state as input. It returns the
        cost of a path between start and goal and the number of nodes expanded.

        If a solution isn't found, it returns -1 for the cost.
        """
                                
        self.OPEN = []
        self.CLOSED = {}

        heapq.heappush(self.OPEN, start)
        node_expanded = 0

        while self.OPEN:
            n = heapq.heappop(self.OPEN)
            node_expanded += 1
            
            if n.__eq__(goal):   # goal found
                return n.get_cost(), node_expanded
            
            self.CLOSED[n.state_hash()] = n
            # Loop through the node’s children
            for childn in self.map.successors(n):
                if childn.state_hash() in self.CLOSED:
                    continue

                # If it is already in CLOSED, skip it                
                if childn in self.OPEN:
                    orig = self.OPEN[self.OPEN.index(childn)].get_g()
                    if orig > childn.get_g():
                        self.OPEN[self.OPEN.index(childn)].set_g(childn.get_g())
                        h_orig = self.h_value(self.OPEN[self.OPEN.index(childn)], goal)
                        f_val = childn.get_g() + h_orig
                        self.OPEN[self.OPEN.index(childn)].set_cost(f_val)
                        heapq.heapify(self.OPEN)                 

                else:
                    # If it isn’t in the open set, calculate the G and H score for n’
                    f_val = childn.get_g() + self.h_value(childn, goal) 
                    childn.set_cost(f_val)   
                    heapq.heappush(self.OPEN, childn)

        return -1, 0
