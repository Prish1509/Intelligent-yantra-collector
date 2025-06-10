# Filename: l2.py
import heapq
# Constant costs for yantra and exit cells
YANTRA_COST = 0
EXIT_COST = 0
TRAP_COST = 99999

class YantraCollector:
    """
    YantraCollector class to solve the yantra collection puzzle with cost-based movement.
    The player must collect all yantras sequentially and reach the exit.
    """
    
    def __init__(self, grid):
        """
        Initializes the game with the provided grid.

        Args:
            grid (list of list of str): The grid representing the puzzle.
        """
        self.grid = grid
        self.n = len(grid)
        self.start = self.find_position('P')
        self.exit = None
        self.yantras = self.find_all_yantras()
        self.revealed_yantra = self.find_position('Y1')
        ## use these variables if needed
        self.collected_yantras = 0
        # self.total_frontier_nodes = 0
        # self.total_explored_nodes = 0
        # self.total_cost = 0
        self.cost_map = self.initialize_cost_map()
        
    def initialize_cost_map(self):
        """
        Initializes a dictionary mapping each position to its movement cost.
        """
        cost_map = {}
        for i in range(self.n):
            for j in range(self.n):
                cell_value = self.grid[i][j]
                if isinstance(cell_value, int):  
                    cost_map[(i, j)] = cell_value
                elif cell_value == 'P':
                    cost_map[(i, j)] = 0
                elif cell_value.startswith('Y'):  
                    cost_map[(i, j)] = YANTRA_COST
                elif cell_value == 'E':  
                    cost_map[(i, j)] = EXIT_COST
                elif cell_value == 'T':
                    cost_map[(i, j)] = TRAP_COST 
                # Walls ('#') are ignored, no need to assign them a cost.
        return cost_map
    
    def find_position(self, symbol):
        """
        Finds the position of a given symbol in the grid.

        Args:
            symbol (str): The symbol to locate.

        Returns:
            tuple or None: The position of the symbol, or None if not found.
        """
        for i in range(self.n):
            for j in range(self.n):
                if self.grid[i][j] == symbol:
                    return (i, j)
        return None

    def find_all_yantras(self):
        """
        Finds and stores the positions of all yantras in the grid.

        Returns:
            dict: A dictionary mapping yantra numbers to their positions.
        """
        positions = {}
        for i in range(self.n):
            for j in range(self.n):
                if isinstance(self.grid[i][j], str) and self.grid[i][j].startswith('Y'):
                    positions[int(self.grid[i][j][1:])] = (i, j)
                elif self.grid[i][j] == 'E':
                    self.exit = (i, j)
        return positions

    def reveal_next_yantra_or_exit(self):
        """
        Reveals the next yantra in sequence or the exit when all yantras are collected.
        """
        self.collected_yantras += 1
        if self.collected_yantras + 1 in self.yantras:
            self.revealed_yantra = self.yantras[self.collected_yantras + 1]
        elif self.collected_yantras == len(self.yantras):
            self.revealed_yantra = self.exit
        else:
            self.revealed_yantra = None

        return self.revealed_yantra

    def goal_test(self, position):
        """
        Checks if the given position matches the currently revealed yantra or exit.
        """
        return position == self.revealed_yantra
        # Returns True if the current position is the revealed yantra (goal), otherwise False.


    def get_neighbors(self, position):
        """
        Generates valid neighboring positions for the given position.
        Each move has an associated movement cost, with yantras and exit having a fixed cost.
        """
        row, col = position  # Extract row and column values of the current position.
    
        # Possible movement directions: Up, Right, Down, Left
        directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]  
        
        neighbors = []  # List to store valid neighboring positions and their movement costs.

        for drow, dcol in directions:
            nrow, ncol = row + drow, col + dcol  # Compute new position after applying the direction.

            # Check if the new position is within grid boundaries and not a blocked cell ('#').
            if 0 <= nrow < self.n and 0 <= ncol < self.n and self.grid[nrow][ncol] != '#':  
                cost = self.cost_map.get((nrow, ncol))  # Get movement cost for this new position.
                neighbors.append((cost, (nrow, ncol)))  # Add the new position along with its cost to the neighbors list.

        return neighbors  # Return the list of valid neighboring positions with their associated movement costs.
                
    
    def ucs(self):
        """
        Performs Uniform Cost Search (UCS) to find the path to the goal.
        """
        # Initialize the frontier with the starting position and a cost of 0
        frontier = [(0, self.start)]  # Priority queue-like list (cost, position)
        front = [self.start]  # Tracks positions currently in the frontier
        explored = set()  # Set to store visited nodes

        total_frontier_nodes = 1  # Count of nodes in the frontier
        total_explored_nodes = 0  # Count of nodes explored

        while frontier:
            # Sort the frontier to always pick the node with the lowest cost 
            frontier.sort()  
            cost, position = frontier.pop(0)  # Remove the node with the lowest cost
            total_frontier_nodes -= 1  # Reduce frontier count as we remove a node

            # Remove the position from the `front` list
            for tup in front:
                if tup == position:
                    front.remove(tup)

            # Skip if the node was already explored
            if position in explored:
                continue

            # Mark the current node as explored
            explored.add(position)
            total_explored_nodes += 1

            # Check if the current position is the goal
            if self.goal_test(position):
                return total_frontier_nodes, total_explored_nodes, cost  

            # Check if there are no neighbors 
            if self.get_neighbors(position) == None:
                return total_frontier_nodes, total_explored_nodes, None

            # Process each neighbor
            for move_cost, neighbor in self.get_neighbors(position):
                if neighbor not in explored:
                    new_cost = cost + move_cost  # Calculate the new path cost

                    # If the neighbor is not in the frontier, add it
                    if neighbor not in front:
                        frontier.append((new_cost, neighbor))
                        front.append(neighbor) 
                        total_frontier_nodes += 1
                    else:
                        # If the new cost is lower than the previously recorded cost, update it
                        if new_cost < move_cost:
                            frontier.pop(move_cost, neighbor)
                            frontier.append((new_cost, neighbor))

        # If no solution is found, return None for cost
        return total_frontier_nodes, total_explored_nodes, None

    def heuristic(self, position):
        """
        Defines a heuristic function to estimate the cost from the current position to the goal-not necessarily admissible.
        """
        d=self.initialize_cost_map()
        count = 0  # Variable to accumulate total movement costs of neighbors
        neighbors =self.get_neighbors(position)  # Retrieve valid neighboring positions

        # Loop through neighbors and sum their movement costs
        for cost, neighbor in neighbors:
            count += cost

        # Compute the average movement cost of neighboring positions
        aver = ((count)// len(neighbors)) + d[position] 
        return aver  # Return the computed heuristic value

    
    def gbfs(self):
        """
        Performs Greedy Best-First Search (GBFS) to find the path to the goal.
        """

        total_frontier_nodes = 1  # Count of nodes added to the frontier
        total_explored_nodes = 0  # Count of nodes explored

        # Initialize the frontier with the starting position and its heuristic value
        frontier = [(0,0, self.start)]  # Priority queue-like list
        explored = []  # List to track visited nodes

        while frontier:
           
            frontier.sort()  # Sort the frontier based on the heuristic value 
            heu,cost, position = frontier.pop(0)  # Remove the node with the lowest heuristic value
            total_frontier_nodes -= 1  # Decrease frontier count

            # Skip if the node was already explored
            if position in explored:
                continue

            # Mark the current node as explored
            explored.append(position)
            total_explored_nodes += 1

            # Check if the current position is the goal
            if self.goal_test(position):
                return total_frontier_nodes, total_explored_nodes, cost  

            # Check if there are no neighbors
            if self.get_neighbors(position) == None:
                return total_frontier_nodes, total_explored_nodes, None

            # Process each neighbor
            for move_cost, neighbor in self.get_neighbors(position):
                if neighbor not in explored:
                    # Add neighbor to the frontier with its heuristic value
                    frontier.append((cost+self.heuristic(neighbor),move_cost+cost, neighbor))
                    total_frontier_nodes += 1

        # If no solution is found, return None for cost
        return total_frontier_nodes, total_explored_nodes, None


    def a_star(self):
        """
        Performs A* Search to find the optimal path to the goal.
        """
        frontier = [(0 + self.heuristic(self.start), 0, self.start)]  # List storing (f, cost, position)
        explored = []
        total_frontier_nodes = 1
        total_explored_nodes = 0
        
        while frontier:
            frontier.sort(key=lambda x: x[0])  # Sort by f-value
            f, cost, position = frontier.pop(0)  # Remove the best node
            total_frontier_nodes-=1
            if self.goal_test(position):
                return total_frontier_nodes, total_explored_nodes, cost
            
            if self.get_neighbors(position)==None:
                return total_frontier_nodes, total_explored_nodes,None

            if position in explored:
                continue
            
            explored.append(position)
            total_explored_nodes += 1
            
            for move_cost,neighbor in self.get_neighbors(position):
                if neighbor not in explored:
                    frontier.append((cost + move_cost + self.heuristic(neighbor), cost + move_cost, neighbor))
                    total_frontier_nodes += 1
        
        return total_frontier_nodes,total_explored_nodes, None  # No solution found
    
    def a_star(self):
        """
        Performs A* Search to find the optimal path to the goal.
        """
        # f = g + h (total estimated cost), g = actual cost from start, h = heuristic estimate
        frontier = [(0 + self.heuristic(self.start), 0, self.start)]  
        explored = []  # List to track visited nodes

        total_frontier_nodes = 1  # Count of nodes in the frontier
        total_explored_nodes = 0  # Count of nodes explored
        
        while frontier:
            # Sort the frontier based on f-value 
            frontier.sort(key=lambda x: x[0])  
            f, cost, position = frontier.pop(0)  # Remove the node with the lowest f-value
            total_frontier_nodes -= 1  # Decrease frontier count

            # Check if the goal has been reached
            if self.goal_test(position):
                return total_frontier_nodes, total_explored_nodes, cost  

            # Check if there are no neighbors (dead-end scenario)
            if self.get_neighbors(position) == None:
                return total_frontier_nodes, total_explored_nodes, None

            # Skip if the node has already been explored
            if position in explored:
                continue
            
            # Mark the current node as explored
            explored.append(position)
            total_explored_nodes += 1

            # Process each neighboring node
            for move_cost, neighbor in self.get_neighbors(position):
                if neighbor not in explored:
                    # Compute new cost values
                    new_g = cost + move_cost  # Actual cost from start node (g)
                    new_f = new_g + self.heuristic(neighbor)  # Total estimated cost (f = g + h)
                    
                    # Add the new node to the frontier
                    frontier.append((new_f, new_g, neighbor))
                    total_frontier_nodes += 1
        
        # If no solution is found, return None for cost
        return total_frontier_nodes, total_explored_nodes, None  


    def solve(self, strategy):
        """
        Solves the yantra collection puzzle using the specified strategy.
        """

        total_frontier_nodes = 0  # Total number of nodes added to the frontier across all searches
        total_explored_nodes = 0  # Total number of nodes explored
        total_cost = 0  # Total cost to collect all yantras and reach the exit

        # Continue searching until all yantras are collected and the exit is reached
        while self.revealed_yantra:
            # Choose the search strategy based on the input parameter
            if strategy == "UCS":
                result = self.ucs()  # Uniform Cost Search
            elif strategy == "GBFS":
                result = self.gbfs()  # Greedy Best-First Search
            elif strategy == "A*":
                result = self.a_star()  # A* Search
            else:
                return None  # Invalid strategy; no solution found

            f_nodes, e_nodes, cost = result
            total_frontier_nodes += f_nodes 
            total_explored_nodes += e_nodes 

            # If no valid path is found, return None (no solution)
            if cost is None:
                return None

            total_cost += cost  

            # Update the starting position to the last collected yantra
            self.start = self.revealed_yantra

            # Reveal the next yantra or the exit for the next phase of search
            self.reveal_next_yantra_or_exit()

        # Return once all yantras are collected and exit is reached
        return total_frontier_nodes, total_explored_nodes, total_cost

if __name__ == "__main__":
    grid = [
        ['P',2 , '#', 5, 'Y2'],
        ['T', 2, 3, '#', 1],
        [0, 7, 'Y1', 4, 2],
        ['#', 'T', 2, 1, 3],
        [1, 3, 0, 2, 'E']
    ]

    game = YantraCollector(grid)
    strategy = "GBFS"  # or "UCS" or "GBFS"
    result = game.solve(strategy)
    
    if result:
        total_frontier_nodes, total_explored_nodes, total_cost = result
        print("Total Frontier Nodes:", total_frontier_nodes)
        print("Total Explored Nodes:", total_explored_nodes)
        print("Total Cost:", total_cost)
    else:
        print("No solution found.")
