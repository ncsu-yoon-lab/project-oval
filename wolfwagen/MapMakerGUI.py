import tkinter as tk 
from tkinter import *

# Creating the window
root = Tk()
root.title('Map Maker')
root.iconbitmap('c:/Users/malin/Downloads/plus.ico')
root.geometry("800x600")

# Setting the w and h and initial x and y
w = 600
h = 400
x = w//2
y = h//2

# Getting current_x and current_y which change as the roads are placed
current_x = 0
current_y = 0

# Keeps track of last type of road placed
past_char = ''

# Keeping track of the roads placed with location and how many openings it has 
# (intersection = [1 , 1 , 1] , vertical road = [0 , 1 , 0] , horizontal road = [0 , 1 , 0]) 
# [x , y , [ 0 or 1 , 0 or 1, 0 or 1,]]
roads = []

# Storing the start and end locations for finding the best route
start_end = []

# Initiaizing speed
speed = 1

# Creating a white canvas
my_canvas = Canvas(root , width = w , height = h , bg = "white")
my_canvas.pack(pady = 20)

# Setting up the label
label = Label(root , text = "")
label.pack(pady = 20)

# Changes label to display x and y positions everytime an arrow key is pressed
def location(x_pos , y_pos):
    label.config(text = "X: " + str(x_pos) + "\nY: " + str(y_pos))

# Commands when arrow keys are hit and changes speed with + and -
def left(event):
    global speed
    x = -1 * speed
    y = 0 * speed
    my_canvas.move(my_image , x , y)
    global current_x
    global current_y
    current_x += x
    current_y += y
    location(current_x , current_y)

def right(event):
    global speed
    x = 1 * speed
    y = 0 * speed
    my_canvas.move(my_image , x , y)
    global current_x
    global current_y
    current_x += x
    current_y += y
    location(current_x , current_y)

def up(event):
    global speed
    x = 0 * speed
    y = -1 * speed
    my_canvas.move(my_image , x , y)
    global current_x
    global current_y
    current_x += x
    current_y += y
    location(current_x , current_y)

def down(event):
    global speed
    x = 0 * speed
    y = 1 * speed
    my_canvas.move(my_image , x , y)
    global current_x
    global current_y
    current_x += x
    current_y += y
    location(current_x , current_y)

# Adds road type and position to roads
def record(x_pos , y_pos , road_type):
    global roads
    global opens
    if road_type == 'i':
        opens = [1 , 1 , 1]
    if road_type == 'v' or road_type == 'h':
        opens = [0 , 1 , 0]
    if road_type == '':
        opens = [0 , 0 , 0]
    roads.append([ x_pos , y_pos , opens])
    print(roads)

# All the key commands
# i = intersection
# v = vertical road
# h = horizontal road
# + = move 10 pixels at a time
# - = move 1 pixel at a time
# s = starting position
# e = ending position
# g = create fastest route
def key(event):
    global my_image
    global past_char
    global speed
    global start_end
    if event.char == 'i':
        record(current_x , current_y , past_char)
        past_char = event.char
        print("4 Way Intersection")
        img = PhotoImage(file = 'c:/Users/malin/Downloads/Coding Projects/Paths/Intersection.png')
        my_image = my_canvas.create_image(current_x , current_y , image = img)
    if event.char == 'v':
        record(current_x , current_y , past_char)
        past_char = event.char
        print("Vertical Road")
        img = PhotoImage(file = 'c:/Users/malin/Downloads/Coding Projects/Paths/Vertical.png')
        my_image = my_canvas.create_image(current_x , current_y , image = img) 
    if event.char == 'h':
        record(current_x , current_y , past_char)
        past_char = event.char
        print("Horizontal Road")
        img = PhotoImage(file = 'c:/Users/malin/Downloads/Coding Projects/Paths/Horizontal.png')
        my_image = my_canvas.create_image(current_x , current_y , image = img)
    if event.char == 's':
        print("Starting Point")
        my_image = my_canvas.create_oval(current_x , current_y , current_x + 10 , current_y + 10 , width = 3)
    if event.char == 'e':
        start_end.append([current_x , current_y])
        print("Ending Point")
        my_image = my_canvas.create_oval(current_x , current_y , current_x + 10 , current_y + 10 , width = 3)
    if event.char == 'g':
        start_end.append([current_x , current_y])
        print("Finding Fastest Route...")
        route_planner(start_end)
    if event.char == '-':
        speed = 1
        print("Slowing Down")
    if event.char == '+':
        speed = 10
        print("Speeding Up")
    root.mainloop()

# Finds the best route after being given the start and end locations
def route_planner(df):
    start_x = df[0][0]
    start_y = df[0][1]
    end_x = df[1][0]
    end_y = df[1][1]
    
# Binds for getting all the inputs from the keyboard
root.bind("<Left>" , left)
root.bind("<Right>" , right)
root.bind("<Up>" , up)
root.bind("<Down>" , down)
root.bind("<Key>" , key)

root.mainloop()