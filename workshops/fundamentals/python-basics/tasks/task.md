# Factory System Update

A smart factory uses robots from different manufacturers to operate its production lines.
The control system stores:
    • A list of robot manufacturers currently used in the factory.
    • A grid that represents robot positions on the factory floor.
During a system update, the factory management makes the following changes:
    1. A new robot manufacturer joins the factory.
    2. One old manufacturer is removed from the system.
    3. The manufacturers list must be reorganized alphabetically for better management.
At the same time, the factory layout needs maintenance:
    • The system must check a specific robot position in the grid.
    • One robot in the grid is replaced with a new version due to maintenance.
After completing all updates, the system should display:
    • The updated list of manufacturers
    • The total number of manufacturers
    • A selected manufacturer by position
    • The selected robot from the grid
    • The final updated factory layout
Your task is to write a Python program that performs these updates and displays the final results clearly.

## Part 1: Manufacturers List

### Step 1 — Create the Initial List
Create a list called:
manufacturers
With these initial values:
    • "ABB"
    • "KUKA"
    • "FANUC"
    • "YASKAWA"

### Step 2 — Update the List
Do the following:
    1. Add a new manufacturer:
"Universal Robots"
    2. Remove this manufacturer:
"FANUC"
    3. Sort the list alphabetically.

### Step 3 — Print the Following
Print:
    • The full updated list
    • The total number of manufacturers
    • The manufacturer at index 1
    • The last manufacturer in the list

## Part 2: Factory Layout Grid

The factory floor is represented as a 2D list (a list inside a list).

### Step 4 — Create the Initial Grid
Create a variable called:
factory_grid
With this structure:
[
    ["R1", "R2", "R3"],
    ["R4", "R5", "R6"],
    ["R7", "R8", "R9"]
]
Each inner list represents one production line.

### Step 5 — Access a Specific Robot
Access the robot located at:
    • Row 1
    • Column 2
Store it in a variable called:
selected_robot

### Step 6 — Replace a Robot
Replace robot "R5" with "R5_NEW".

### Step 7 — Print the Following
Print:
    • The selected robot
    • The updated factory grid

