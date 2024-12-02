
import tkinter as tk
import heapq
import random
from tkinter import messagebox
from tkinter import ttk

# Initialize the main window
root = tk.Tk()
root.title("Factory Layout Simulator with Multiple AGVs")
root.geometry("700x700")

# Add a canvas for drawing the grid
canvas = tk.Canvas(root, width=500, height=500, bg="white")
canvas.pack()

# Define grid size and cell size
grid_size = 15  # 10x10 grid
cell_size = 500 // grid_size

# Cell states and pathfinding grid
cell_states = {}
path_grid = [[0] * grid_size for _ in range(grid_size)]

# Define AGV details
agvs = {
    1: {
        "start": None,
        "position": None,
        "pickup": None,
        "dropoff": None,
        "path": [],
        "pickup_color": "green",
        "dropoff_color": "red",
        "task": "pickup",  # Can be "pickup" or "dropoff"
        "steps": 0,
        "total_cost": 0,
    },
    2: {
        "start": None,
        "position": None,
        "pickup": None,
        "dropoff": None,
        "path": [],
        "pickup_color": "purple",
        "dropoff_color": "yellow",
        "task": "pickup",  # Can be "pickup" or "dropoff"
        "steps": 0,
        "total_cost": 0,
    },
}

# Define available algorithms
algorithms = ["A*", "Dijkstra", "BFS"]
selected_algorithm = tk.StringVar(value=algorithms[0])  # Default to A*

# Dropdown to select the algorithm
algorithm_label = tk.Label(root, text="Select Algorithm:")
algorithm_label.pack(pady=10)
algorithm_menu = tk.OptionMenu(root, selected_algorithm, *algorithms)
algorithm_menu.pack()


def random_layout():
    # Clear existing layout
    canvas.delete("all")
    draw_grid()  # Redraw grid
    global path_grid, cell_states
    cell_states = {}
    path_grid = [[0] * grid_size for _ in range(grid_size)]

    # Randomly place walls
    wall_count = int(grid_size * grid_size * 0.2)  # ~20% of the grid as walls
    for _ in range(wall_count):
        while True:
            x, y = random.randint(0, grid_size - 1), random.randint(0, grid_size - 1)
            if path_grid[x][y] == 0:  # Only place wall if cell is empty
                path_grid[x][y] = 1
                cell_states[(x, y)] = "obstacle"
                canvas.create_rectangle(
                    x * cell_size, y * cell_size,
                    (x + 1) * cell_size, (y + 1) * cell_size,
                    fill="gray"
                )
                break

    # Randomly place pickup and drop-off points for each AGV
    for agv_id, agv in agvs.items():
        for point_type in ["pickup", "dropoff"]:
            while True:
                x, y = random.randint(0, grid_size - 1), random.randint(0, grid_size - 1)
                if path_grid[x][y] == 0:  # Ensure the cell is not a wall or already occupied
                    agv[point_type] = (x, y)
                    color = agv["pickup_color"] if point_type == "pickup" else agv["dropoff_color"]
                    canvas.create_rectangle(
                        x * cell_size, y * cell_size,
                        (x + 1) * cell_size, (y + 1) * cell_size,
                        fill=color
                    )
                    break


# Function to draw the grid
def draw_grid():
    for i in range(grid_size):
        for j in range(grid_size):
            x0, y0 = i * cell_size, j * cell_size
            x1, y1 = x0 + cell_size, y0 + cell_size
            canvas.create_rectangle(x0, y0, x1, y1, fill="white", outline="black")

# Draw initial grid
draw_grid()

# Place AGV on the grid based on the AGV ID
def place_agv(event, agv_id):
    x, y = event.x // cell_size, event.y // cell_size
    if x >= grid_size or y >= grid_size:
        return  # Ignore clicks outside the grid
    agvs[agv_id]["start"] = (x, y)  # Set the start position

    if agvs[agv_id]["position"]:
        canvas.delete(agvs[agv_id]["position"])

    agvs[agv_id]["position"] = canvas.create_oval(
        x * cell_size, y * cell_size,
        (x + 1) * cell_size, (y + 1) * cell_size,
        fill="black" if agv_id == 1 else "orange"
    )

# Set pick-up and drop-off points with Shift-click
def set_pickup_dropoff(event, agv_id, point_type):
    x, y = event.x // cell_size, event.y // cell_size
    if x >= grid_size or y >= grid_size:
        return  # Ignore clicks outside the grid
    agvs[agv_id][point_type] = (x, y)
    color = agvs[agv_id]["pickup_color"] if point_type == "pickup" else agvs[agv_id]["dropoff_color"]
    canvas.create_rectangle(
        x * cell_size, y * cell_size,
        (x + 1) * cell_size, (y + 1) * cell_size,
        fill=color
    )
# Mark cell with obstacles (walls) using left-click
def mark_obstacle(event):
    x, y = event.x // cell_size, event.y // cell_size
    if x >= grid_size or y >= grid_size:
        return  # Ignore clicks outside the grid
    cell_id = (x, y)
    if cell_id not in cell_states:
        cell_states[cell_id] = "obstacle"
        path_grid[x][y] = 1
        canvas.create_rectangle(
            x * cell_size, y * cell_size,
            (x + 1) * cell_size, (y + 1) * cell_size,
            fill="gray"
        )
    else:
        cell_states.pop(cell_id)
        path_grid[x][y] = 0
        canvas.create_rectangle(
            x * cell_size, y * cell_size,
            (x + 1) * cell_size, (y + 1) * cell_size,
            fill="white"
        )

# Heuristic function for A* (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Reconstruct path from start to goal
def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

# Get valid neighbors for pathfinding
def get_neighbors(cell):
    neighbors = []
    x, y = cell
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < grid_size and 0 <= ny < grid_size and path_grid[nx][ny] == 0:
            neighbors.append((nx, ny))
    return neighbors

# Pathfinding algorithms
def astar(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []

def dijkstra(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    cost = {start: 0}

    while open_set:
        current_cost, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current):
            new_cost = current_cost + 1

            if neighbor not in cost or new_cost < cost[neighbor]:
                came_from[neighbor] = current
                cost[neighbor] = new_cost
                heapq.heappush(open_set, (new_cost, neighbor))

    return []
# Function to perform BFS for pathfinding
def bfs(start, goal):
    from collections import deque
    queue = deque([start])
    came_from = {}
    visited = set()
    visited.add(start)

    while queue:
        current = queue.popleft()

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)

    return []  # Return empty if no path is found

# Add BFS to the list of available algorithms
algorithms.append("BFS")


# Add AGV trackers in the top-left corner
top_left_frame = tk.Frame(root)
top_left_frame.place(x=10, y=10)  # Position in the top-left corner

# AGV UI elements for trackers
step_label_vars = {}
next_point_vars = {}
progress_labels = {}
progress_bars = {}
#
# for agv_id in agvs:
#     agv_frame = tk.Frame(top_left_frame)
#     agv_frame.pack(pady=5, anchor="w")
#
#     tk.Label(agv_frame, text=f"AGV {agv_id}").pack(anchor="w")
#     step_label_vars[agv_id] = tk.StringVar(value="Steps: 0")
#     tk.Label(agv_frame, textvariable=step_label_vars[agv_id]).pack(anchor="w")
#     next_point_vars[agv_id] = tk.StringVar(value="Next: None")
#     tk.Label(agv_frame, textvariable=next_point_vars[agv_id]).pack(anchor="w")
#
#     progress_labels[agv_id] = tk.Label(agv_frame, text=f"Progress: 0%")
#     progress_labels[agv_id].pack(anchor="w")
#     progress_bars[agv_id] = ttk.Progressbar(agv_frame, orient="horizontal", length=200, mode="determinate")
#     progress_bars[agv_id].pack(anchor="w")
#
# # Update trackers dynamically
def update_ui(agv_id):
    agv = agvs[agv_id]
    step_label_vars[agv_id].set(f"Steps: {agv['steps']}")
    agv["next_point"] = agv["path"][0] if agv["path"] else "Task Complete"
    next_point_vars[agv_id].set(f"Next: {agv['next_point']}")
    if agv["total_cost"] > 0:  # Avoid division by zero
        progress = int((agv["steps"] / agv["total_cost"]) * 100)
        progress_labels[agv_id].config(text=f"Progress: {progress}%")
        progress_bars[agv_id]["value"] = progress

# Simulate AGV movement with UI updates
def move_agv(agv_id):
    agv = agvs[agv_id]

    if not agv["path"]:
        if agv["task"] == "pickup":
            agv["task"] = "dropoff"
            agv["path"] = astar(agv["pickup"], agv["dropoff"]) if selected_algorithm.get() == "A*" else dijkstra(agv["pickup"], agv["dropoff"])
        else:
            update_ui(agv_id)
            print(f"AGV {agv_id} has completed its task!")
            return

    if agv["path"]:
        next_cell = agv["path"].pop(0)
        agv["steps"] += 1
        canvas.coords(
            agv["position"],
            next_cell[0] * cell_size, next_cell[1] * cell_size,
            (next_cell[0] + 1) * cell_size, (next_cell[1] + 1) * cell_size
        )
        update_ui(agv_id)
        root.after(300, move_agv, agv_id)

# Start simulation with total cost calculation
# Start simulation with total cost calculation
def start_simulation():
    for agv_id, agv in agvs.items():
        if not agv["start"] or not agv["pickup"] or not agv["dropoff"]:
            print(f"AGV {agv_id} is missing start, pickup, or drop-off points!")
            continue

        algorithm = selected_algorithm.get()
        if algorithm == "A*":
            path_to_pickup = astar(agv["start"], agv["pickup"])
            path_to_dropoff = astar(agv["pickup"], agv["dropoff"])
        elif algorithm == "Dijkstra":
            path_to_pickup = dijkstra(agv["start"], agv["pickup"])
            path_to_dropoff = dijkstra(agv["pickup"], agv["dropoff"])
        elif algorithm == "BFS":
            path_to_pickup = bfs(agv["start"], agv["pickup"])
            path_to_dropoff = bfs(agv["pickup"], agv["dropoff"])
        else:
            print("Unknown algorithm selected!")
            continue

        agv["total_cost"] = len(path_to_pickup) + len(path_to_dropoff)
        agv["steps"] = 0
        agv["path"] = path_to_pickup
        agv["task"] = "pickup"
        move_agv(agv_id)
# Calculate costs for AGVs
def calculate_cost():
    # Map algorithm names to their respective functions
    algorithm_functions = {"A*": astar, "Dijkstra": dijkstra, "BFS": bfs}
    costs = {algo: {"AGV 1": 0, "AGV 2": 0, "Total": 0} for algo in algorithm_functions}

    for algo, func in algorithm_functions.items():
        for agv_id, agv in agvs.items():
            if not agv["start"] or not agv["pickup"] or not agv["dropoff"]:
                continue

            # Get paths for pickup and drop-off
            path_to_pickup = func(agv["start"], agv["pickup"])
            path_to_dropoff = func(agv["pickup"], agv["dropoff"])

            # Calculate the path cost
            total_cost = len(path_to_pickup) + len(path_to_dropoff)
            costs[algo][f"AGV {agv_id}"] = total_cost
            costs[algo]["Total"] += total_cost

            # Debugging output for paths
            print(f"[{algo}] AGV {agv_id} Pickup Path: {path_to_pickup}")
            print(f"[{algo}] AGV {agv_id} Drop-off Path: {path_to_dropoff}")
            print(f"[{algo}] AGV {agv_id} Total Cost: {total_cost}")

    # Display costs
    cost_message = "Cost Calculation:\n"
    for algo, cost in costs.items():
        cost_message += f"{algo}:\n"
        cost_message += f"  AGV 1 Cost: {cost['AGV 1']}\n"
        cost_message += f"  AGV 2 Cost: {cost['AGV 2']}\n"
        cost_message += f"  Total Cost: {cost['Total']}\n"

    messagebox.showinfo("Cost Calculation", cost_message)

# Horizontal button layout
button_frame = tk.Frame(root)
button_frame.pack(pady=10)

start_button = tk.Button(button_frame, text="Start Simulation", command=start_simulation)
start_button.pack(side="left", padx=5)

# Add a button to trigger random layout generation
random_layout_button = tk.Button(button_frame, text="Random Layout", command=random_layout)
random_layout_button.pack(side="left", padx=5)


calculate_button = tk.Button(button_frame, text="Calculate Total Cost", command=calculate_cost)
calculate_button.pack(side="left", padx=5)

for agv_id in agvs:
    tk.Button(button_frame, text=f"Place AGV {agv_id}",
              command=lambda agv_id=agv_id: canvas.bind("<Button-1>", lambda event, agv_id=agv_id: place_agv(event, agv_id))).pack(side="left", padx=5)

# Add progress bars to the left corner within the top-left tracker frame
progress_labels = {}
progress_bars = {}

for agv_id in agvs:
    agv_frame = tk.Frame(top_left_frame)
    agv_frame.pack(pady=5, anchor="w")

    tk.Label(agv_frame, text=f"AGV {agv_id}").pack(anchor="w")

    # Steps tracker
    step_label_vars[agv_id] = tk.StringVar(value="Steps: 0")
    tk.Label(agv_frame, textvariable=step_label_vars[agv_id]).pack(anchor="w")

    # Next point tracker
    next_point_vars[agv_id] = tk.StringVar(value="Next: None")
    tk.Label(agv_frame, textvariable=next_point_vars[agv_id]).pack(anchor="w")

    # Progress bar label
    progress_labels[agv_id] = tk.Label(agv_frame, text=f"Progress: 0%")
    progress_labels[agv_id].pack(anchor="w")

    # Progress bar
    progress_bars[agv_id] = ttk.Progressbar(agv_frame, orient="horizontal", length=200, mode="determinate")
    progress_bars[agv_id].pack(anchor="w")

# Bind mouse clicks for setting obstacles, pickup, and drop-off points
canvas.bind("<Button-1>", mark_obstacle)
canvas.bind("<Shift-Button-1>", lambda event: set_pickup_dropoff(event, 1, "pickup"))
canvas.bind("<Shift-Button-3>", lambda event: set_pickup_dropoff(event, 1, "dropoff"))
canvas.bind("<Control-Button-1>", lambda event: set_pickup_dropoff(event, 2, "pickup"))
canvas.bind("<Control-Button-3>", lambda event: set_pickup_dropoff(event, 2, "dropoff"))

# Run the GUI
root.mainloop()
