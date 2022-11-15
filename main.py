import tkinter as tk
import numpy as np
import astar

title = "A* Path Finding"
width, height = 550, 550
window = tk.Tk()
window.title(title)
window.geometry(f"{width}x{height}")

grid_w, grid_h = 20, 20
grid = [
    [0 if (y == grid_h - 1 and x == grid_w - 1) or (x == 0 and y == 0) else (
        0 if np.random.randint(0, 10) < 8 else 1)
     for y in range(grid_h)] for x in range(grid_w)]

rect_size = width / grid_w
start = (0, 0)
goal = (grid_w - 1, grid_h - 1)

utils = astar.AStarUtils(grid_w, grid_h, grid)

cell_edit_value = False

show_f_var = tk.IntVar()
edit_states_var = tk.IntVar()
grid_size_var = tk.StringVar()


def start_func():
    path, cells = astar.a_star(start, goal, utils)
    if path:
        draw_grid(path=path, cells=cells)
    else:
        draw_grid(found=False)


def generate(random=False):
    global grid
    global start
    global goal
    global utils
    global rect_size

    grid = [
        [0 if (y == goal[1] and x == goal[0]) or (x == start[0] and y == start[1]) else (
            0 if not random else (0 if np.random.randint(0, 10) < 8 else 1))
         for y in range(grid_h)] for x in range(grid_w)]

    rect_size = width / grid_w

    utils = astar.AStarUtils(grid_w, grid_h, grid)


def handle_start(event):
    start_func()


def handle_clear():
    generate()
    draw_grid()


def handle_random():
    generate(random=True)
    draw_grid()


def handle_grid_resize():
    global utils, grid_w, grid_h, goal, start
    value = grid_size_var.get()
    if value == "Grid Size S":
        grid_w, grid_h = (20, 20)
    elif value == "Grid Size M":
        grid_w, grid_h = (30, 30)
    elif value == "Grid Size L":
        grid_w, grid_h = (40, 40)

    start = (0, 0)
    goal = (grid_w - 1, grid_h - 1)
    generate(random=True)
    draw_grid()


def create_menubar():
    menubar = tk.Menu()

    # Declare file and edit for showing in menu bar
    file = tk.Menu(menubar, tearoff=False)
    edit = tk.Menu(menubar, tearoff=False)

    file.add_command(label="Run", command=start_func)

    edit.add_radiobutton(label="Grid Size S", variable=grid_size_var, command=handle_grid_resize)
    edit.add_radiobutton(label="Grid Size M", variable=grid_size_var, command=handle_grid_resize)
    edit.add_radiobutton(label="Grid Size L", variable=grid_size_var, command=handle_grid_resize)
    edit.add_separator()
    edit.add_checkbutton(label="Edit States", variable=edit_states_var)
    edit.add_checkbutton(label="Show f(n)", variable=show_f_var)

    # Display file and edit declared in previous step
    menubar.add_cascade(label="File", menu=file)
    menubar.add_cascade(label="Edit", menu=edit)

    # Display of menu bar in the app
    window.config(menu=menubar)


def create_toolbar():
    toolbar = tk.Label(window)
    toolbar.pack(side=tk.TOP, fill=tk.X)

    run_btn = tk.Button(toolbar, text="Run", command=start_func)
    run_btn.grid(row=0, column=0, padx=2)

    clear_btn = tk.Button(toolbar, text="Clear", command=handle_clear)
    clear_btn.grid(row=0, column=1, padx=2)

    rand_btn = tk.Button(toolbar, text="Random", command=handle_random)
    rand_btn.grid(row=0, column=2, padx=2)


create_menubar()
create_toolbar()


def click_callback(event):
    global cell_edit_value
    global edit_states_var
    global goal, start

    x, y = int(event.x / rect_size), int(event.y / rect_size)
    if not utils.is_valid(x, y):
        return

    if edit_states_var.get():
        if (x, y) == goal:
            temp = start
            start = goal
            goal = temp
        grid[x][y] = 0
        start = (x, y)
        draw_grid()
        return

    if (x, y) == start:
        start_func()
    else:
        grid[x][y] = 0 if grid[x][y] == 1 else 1
        cell_edit_value = grid[x][y]
        draw_grid()


def click2_callback(event):
    global edit_states_var
    global goal, start
    if not edit_states_var.get():
        return

    x, y = int(event.x / rect_size), int(event.y / rect_size)
    if not utils.is_valid(x, y):
        return
    if (x, y) == start:
        temp = start
        start = goal
        goal = temp
    grid[x][y] = 0
    goal = (x, y)
    draw_grid()


def motion_callback1(event):
    global cell_edit_value
    global edit_states_var
    if edit_states_var.get():
        return
    x, y = int(event.x / rect_size), int(event.y / rect_size)
    if not utils.is_valid(x, y):
        return
    if (x, y) != start:
        grid[x][y] = cell_edit_value
        draw_grid()


c = tk.Canvas(window, bg="black", height=height, width=width)
c.bind("<Button-1>", click_callback)
c.bind("<Button-3>", click2_callback)
c.bind("<B1-Motion>", motion_callback1)


def draw_grid(path=[], cells=[], found=True):
    c.delete("all")
    for x in range(grid_w):
        for y in range(grid_h):
            color = "white" if found else "red"
            if (x, y) == start:
                color = "green"
            elif (x, y) == goal:
                color = "red"
            elif grid[x][y] == 1:
                color = "black"
            elif (x, y) in path:
                color = "blue"

            c.create_rectangle((x * rect_size, y * rect_size, (x * rect_size) + rect_size, (y * rect_size) + rect_size),
                               fill=color)
            if cells:
                f_value = cells[x][y].f_value
                if f_value < astar.FLOAT_MAX and show_f_var.get():
                    c.create_text((x * rect_size + (rect_size / 2), y * rect_size + (rect_size / 2)),
                                  text="{0:.1f}".format(f_value), fill="orange")


generate(random=True)
draw_grid()
c.pack()
# Start the event loop.
window.mainloop()
