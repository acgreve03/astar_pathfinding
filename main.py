import pygame
from queue import PriorityQueue

WIDTH = 800     #setting width of window size
WINDOW = pygame.display.set_mode((WIDTH, WIDTH))   #initializing pygame window
pygame.display.set_caption("astar pathfinding visualization")   #Setting window caption

#assigning colors to their respective RGB values
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (0, 255, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

#class representing each node, or cube, in the GUI
class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        #x and y are used to keep track of the coordinate position of each node, so that pygame knows where to put each node/cube
        self.x = row * width 
        self.y = col * width
        #to start, will have all white cubes
        self.color = WHITE
        #keeps track of current node's neighbors, for the algorithm
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    '''the following methods are implemented to tell us the state of a given node...'''

    #returns the row, col position of a given node
    def get_pos(self):
        return self.row, self.col
    
    #tells us if a given node has been 'visited'... returns true if the node color is red
    def is_closed(self):
        return self.color == RED
    
     #tells us if a given node is unvisited... returns true if the node color is green
    def is_open(self):
        return self.color == GREEN
    
    #tells us if the given node is a barrier/wall.
    def is_barrier(self):
        return self.color == BLACK
    
     #tells us if the given node is the starting node.
    def is_start(self):
        return self.color == ORANGE
    
    #tells us if the given node is the ending node.
    def is_end(self):
        return self.color == TURQUOISE
    
    #resets a given node
    def reset(self):
         self.color = WHITE
    
    #changes the state of a node to 'visited'
    def make_closed(self):
        self.color = RED

    #changes the state of a node to 'unvisited'
    def make_open(self):
        self.color = GREEN

    #changes the state of a node to be a barrier
    def make_barrier(self):
        self.color = BLACK

    #changes the state of a node to be the starting node
    def make_start(self):
        self.color = ORANGE

    #changes the state of a node to be the ending node
    def make_end(self):
        self.color = TURQUOISE

    #changes the state of a node to represent part of the path that was found by the algorithm
    def make_path(self):
        self.color = PURPLE

    #method that actually draws the node on screen... this is where the x,y coordinates come in handy
    #pygame draws from the top left of the screen, so (0,0) is the top-left corner of the screen...
    def draw(self, window):
        pygame.draw.rect(window, self.color, (self.x, self.y, self.width, self.width))

    #updates the neighbors array
    def update_neighbors(self, grid):
        self.neighbors = []
        #if there exists a row below current row, and if the node below is not a barrier...
        if self.row < self.total_rows - 1 and not grid[self.row+1][self.col].is_barrier():  #down
            self.neighbors.append(grid[self.row + 1][self.col]) #append to 'neighbors' array

        #if there exists a row above the current row, and if the node above us is not a barrier...
        if self.row > 0 and not grid[self.row-1][self.col].is_barrier():  #up
            self.neighbors.append(grid[self.row - 1][self.col]) #append to 'neighbors' array

        #if there exists a col to the left the current col, and if the node left of the current node is not a barrier...
        if self.col > 0 and not grid[self.row][self.col-1].is_barrier():  #left
            self.neighbors.append(grid[self.row][self.col-1]) #append to 'neighbors' array

        #if there exists a col to the right the current col, and if the node right of the current node is not a barrier...
        if self.col < self.total_rows - 1 and not grid[self.row][self.col+1].is_barrier():  #right
            self.neighbors.append(grid[self.row][self.col+1]) #append to 'neighbors' array

    '''def __lt__(self, other):
        return False'''


#returns the 'heuristic', or estimate, of how far the current node is to the end node...
def h(p1, p2):  #point 1, point 2
    x1, y1 = p1
    x2, y2 = p2
    return abs(x2-x1) + abs(y2-y1)

def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()

#algorithm function
def astar(draw, grid, start, end):
    count = 0   #keeping track of when a value is put into the PQ... used to break a tie if two values have the same f score
    open_set = PriorityQueue()
    open_set.put((0, count, start)) #adding to the PQ
    came_from = {}  #keeps track of which node the current node came from
    g_score = {node: float("inf") for row in grid for node in row}  #g score is distance to each neighboring node, initializing the g score to infinity first 
    g_score[start] = 0  #since the distance to itself is 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = h(start.get_pos(), end.get_pos())  #f = g + h, so f = 0 + h, = h at start

    #keeps track of elements in the PQ, and lets us check if an element exists in the PQ
    open_set_hash = {start}

    #runs until the set is empty
    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2] #indexing at 2 since set stores the f-score, count, and node... we just want the node
        open_set_hash.remove(current) #takes node that was 'popped' out of the PQ, and we sync it with the set by removing it from the set...

        #if we have found the end node...
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True
        
        #loops thru each neighbor of the current node's neighbors
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1 #currently known shortest distance + 1

            #if a better path to reach the neighbor than before was found...
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score    #update g score 
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())    #update f score 
                
                #adding in the neighbor with better, more optimal path
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()

    #if we did not find a path...
    return False

#creates the grid.
def make_grid(rows, width):
    grid = []
    gap = width // rows     #using integer division to tell us what the distance, in px, should be between each row. i.e. the gap...

    for i in range(rows):   #both loops have same bounds b/c the grid is a square.
        grid.append([])     #appends a new row
        for j in range(rows):
            node = Node(i, j, gap, rows)    #creates a new node object at each col position in the given row.
            grid[i].append(node)    #appends the new node object to the given row after each object initialization...
    
    return grid

#draws grid lines, not the grid itself
def draw_grid(window, rows, width):
    gap = width // rows
    for i in range(rows):
        #draws horizontal grid lines
        pygame.draw.line(window, GREY, (0, i * gap), (width, i * gap))   #(xstart, ystart), (xend, yend)

        #draws vertical grid lines
        for j in range(rows):
            pygame.draw.line(window, GREY, (j * gap, 0), (j * gap, width))  #(xstart, ystart), (xend, yend)
    
#main draw method. draws everything...
def draw(window, grid, rows, width):
    window.fill(WHITE)

    #draws the color for each node
    for row in grid:
        for node in row:
            node.draw(window)

    #draws the grid lines
    draw_grid(window, rows, width)
    #updates each time something new is drawn 
    pygame.display.update()     

#function that returns the position of the mouse when clicked
#this is how the program knows which node changes state when a mouse click event occurs
def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y,x = pos   

    row = y // gap  #y used for row to determine height, gives us row pos
    col = x // gap  #x used for row to determine width, gives us col pos
    return row, col


def main(window, width):
    ROWS = 50   #number of rows
    grid = make_grid(ROWS, width)   #generates grid, gives us 2d array of nodes

    start = None
    end = None

    run = True

    while run:
        draw(window, grid, ROWS, width)     

        #loop thru each event that has occurred
        for event in pygame.event.get():
            if event.type == pygame.QUIT:   #if we quit the program...
                run = False


            if pygame.mouse.get_pressed()[0]: #left mouse
                pos = pygame.mouse.get_pos() #gets current Lmouse pos
                row, col = get_clicked_pos(pos, ROWS, width) #gets the row, col pos of the Lmouse click
                node = grid[row][col] #assigns node to the row, col pos given from the above line of code
                
                #if start and end pos haven't been assigned yet, and if start node isnt the same as end node
                if not start and node != end:
                    start = node
                    start.make_start()

                #if start has been assigned, but not end
                elif not end and node != start:
                    end = node
                    end.make_end()

                #if start and end node have both been assigned
                elif node != end and node != start:
                    node.make_barrier()

            elif pygame.mouse.get_pressed()[2]: #right mouse
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                node.reset()    #resets the node to white

                #resets start node to white
                if node == start:
                    start = None
                
                #resets end node to white
                if node == end:
                    end = None

            
            if event.type == pygame.KEYDOWN:    #if key was pressed on keyboard
                if event.key == pygame.K_SPACE  and start and end:    #if the key was space bar and if start and end have been placed...
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)
                    astar(lambda: draw(window, grid, ROWS, width), grid, start, end)    #using lambda function to call the draw function inside of the astar alg. function

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

                    


    pygame.quit()

main(WINDOW, WIDTH)

