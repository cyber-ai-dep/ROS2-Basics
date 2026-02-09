
# Introduction to Python Programming with Emphasis on Robotics Applications



# Table of Contents

- [Introduction](#introduction)
- [1. Environment & Running Code (Ubuntu + VS Code)](#1-environment--running-code-ubuntu--vs-code)
- [2. Python Variables -- Teaching the Robot to Remember](#2-python-variables----teaching-the-robot-to-remember)
- [3. Python Data Structures -- Lists, Dictionaries, and Tuples](#5-python-data-structures----lists-dictionaries-and-tuples)
- [4. Python Loops -- Teaching the Robot to Repeat Actions](#12-python-loops----teaching-the-robot-to-repeat-actions)
- [5. Python Functions -- Teaching the Robot to Reuse Intelligence](#20-python-functions----teaching-the-robot-to-reuse-intelligence)
- [6. Classes and Objects in Python](#22-classes-and-objects-in-python)
- [7. Python Modules & Imports](#23-python-modules--imports)


## Introduction
This lesson is designed to refresh your knowledge of **Python programming**, with a specific focus on **robotics applications**. Since Python will be used extensively to program and control our robots throughout this course, it is important to revisit the fundamentals from a robotics perspective before moving forward.

Python is one of the most widely used programming languages due to its simplicity and beginner-friendly syntax. At the same time, it is extremely powerful. Python's core functionality can be easily extended by installing and importing external packages, allowing developers to quickly add advanced features without writing everything from scratch. This flexibility makes Python especially suitable for robotics, where rapid development and experimentation are essential.

The **Python Package Index (PyPI)** is the main repository for third-party Python packages. It hosts hundreds of thousands of libraries that provide ready-to-use solutions for tasks such as robotics, computer vision, data processing, and machine learning. These packages can be easily installed using the pip command, enabling developers to quickly build complex systems with minimal effort.

## 1) Environment & Running Code (Ubuntu + VS Code)

### Run a Python file

```bash
python3 your_script.py
```

### Quick interpreter testing

```bash
python3
```

```python
>>> print("test")
>>> exit()
```

---

## Python Variables -- Teaching the Robot to Remember

### Section Objective

By the end of this section, learners will be able to:

- Understand what variables are and why we use them
- Create and update variables in Python
- Use variables to represent **robot states**, **sensor values**, and **commands**
- Write a small practical script using variables

### What Is a Variable?


A **variable** is like a container that stores data. It can be a number, a text, or more complex data types. In most programming languages like C++, you need to declare a variable before using it but in Python, you just give it a name and assign a value to it. Different data types that can be stored in variables are:

- Numbers: different types like integers and floats
- Boolean value: True or False
- Strings: plain texts
- Lists
- Tuples
- Dictionaries

### Variable Naming Rules

- Must start with a letter or `_`
- No spaces
- Case-sensitive (Speed ≠ speed)
- Use meaningful names (important in robotics!)

![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/variableNaming.jpg)


### Numbers and Strings

Let's first talk about the simplest data types in Python: numbers and strings. Suppose we want to create a variable named battery_voltage and assign the value of 10.0 to that variable. Here is how we can do it:

```python
distance = 30.0  # this number is a float
```

- `distance` → variable name
- `=` → assignment operator
- `30.0` → value stored in memory



### Basic Arithmetic

Python supports basic arithmetic operations:

- `+` addition
- `-` subtraction
- `*` multiplication
- `/` division
- `()` grouping
- `**` power

Example:

```python
print(robot_count * (robot_count + broken_ratio))
```

### Modulus Operator

The modulus operator `%` gives the remainder of a division:

```python
print(robot_count % 5)
```

### Built-in Numeric Functions

Python provides built-in functions that work directly with numbers:

- `max()` → returns the larger number
- `min()` → returns the smaller number
- `abs()` → absolute value
- `pow()` → power calculation
- `round()` → rounds a number

Examples:

```python
print(max(robot_count, 8))
print(min(robot_count, 8))
print(abs(-robot_count))
print(pow(robot_count, 2))
print(round(broken_ratio))
```

### Combining Numbers and Strings

To combine numbers with text, we must convert numbers to strings using `str()`.

```python
print(
    "There are " + str(robot_count) +
    " robots in the warehouse, " +
    str(broken_ratio) +
    " of them are malfunctioning."
)
```

Output:
```
There are 12 robots in the warehouse, 0.75 of them are malfunctioning.
```

You can also write this in a simpler way using commas:

```python
print(
    "There are", robot_count,
    "robots in the warehouse,",
    broken_ratio,
    "of them are malfunctioning."
)
```

**Note:** `\n` creates a new line inside a string.

### String Variables

You can store text inside variables:

```python
warehouse_name = "ROBO_HUB"
print("The warehouse name is " + warehouse_name)
```

Since warehouse_name is already a string, we do **not** need `str()` here.

### Accessing Characters in a String

You can access individual characters using square brackets `[]`.

```python
print("First character:", warehouse_name[0])
print("Fourth character:", warehouse_name[3])
```

Indexing in Python **starts from 0**.

### String Functions

#### Changing Case

```python
print(warehouse_name.lower())
print(warehouse_name.upper())
```

#### Checking Case

```python
print(warehouse_name.islower())
print(warehouse_name.isupper())
```

#### Combining String Functions

```python
print(warehouse_name.lower().islower())
```

### String Length

To find the length of a string:

```python
print(len(warehouse_name))
```

### Slicing Strings

To extract part of a string:

```python
print(warehouse_name[0:len(warehouse_name)-1])
```

This prints all characters except the last one.

### Finding Characters

To find the position of a character:

```python
print(warehouse_name.index("R"))
```

### Replacing Text

To replace part of a string:

```python
print(warehouse_name.replace("ROBO", "AUTO"))
```

### Full Code

```python
robot_count = 12
broken_ratio = 0.75

# arithmetic operations
print(robot_count * (robot_count + broken_ratio))

# modulus operator
print(robot_count % 5)

# built-in numeric functions
print(max(robot_count, 8))
print(min(robot_count, 8))
print(abs(-robot_count))
print(pow(robot_count, 2))
print(round(broken_ratio))

# combining numbers with strings
print(
    "There are " + str(robot_count) +
    " robots in the warehouse, " +
    str(broken_ratio) +
    " of them are malfunctioning."
)

print(
    "There are", robot_count,
    "robots in the warehouse,",
    broken_ratio,
    "of them are malfunctioning."
)

# string variables
warehouse_name = "ROBO_HUB"
print("The warehouse name is " + warehouse_name)

# accessing characters
print("First character:", warehouse_name[0])
print("Fourth character:", warehouse_name[3])

# string functions
print(warehouse_name.lower())
print(warehouse_name.islower())
print(warehouse_name.upper().isupper())

# string length
print(len(warehouse_name))

# slicing
print(warehouse_name[0:len(warehouse_name)-1])

# finding characters
print(warehouse_name.index("R"))

# replacing text
print(warehouse_name.replace("ROBO", "AUTO"))
```

---

## Booleans (True and False)

A **Boolean** is a data type that can have **only two values**:

- True
- False

Booleans are commonly used in **decision making**, **conditions**, and **logic**.

### Creating Boolean Variables

You can assign Boolean values directly to variables:

```python
robot_active = True
emergency_stop = False
```

These variables represent logical states, not numbers or text.

### Boolean from Comparisons

Most Boolean values come from **comparisons**.

#### Comparison Operators

| Operator | Meaning |
|----------|---------|
| `==` | equal |
| `!=` | not equal |
| `>` | greater than |
| `<` | less than |
| `>=` | greater than or equal |
| `<=` | less than or equal |

Examples:

```python
battery_level = 80
print(battery_level > 50)   # True
print(battery_level == 100) # False
print(battery_level <= 20)  # False
```

### Boolean with Strings

Booleans also work with strings:

```python
robot_name = "ATLAS"
print(robot_name == "ATLAS")  # True
print(robot_name == "atlas")  # False (case-sensitive)
```

### Logical Operators

Logical operators allow us to **combine conditions**.

#### and

Returns True only if **both conditions are True**.

```python
battery_ok = True
motors_ok = True
print(battery_ok and motors_ok)
```

#### or

Returns True if **at least one condition is True**.

```python
manual_mode = False
remote_control = True
print(manual_mode or remote_control)
```

#### not

Reverses the Boolean value.

```python
sensor_active = True
print(not sensor_active)
```

### Boolean from Functions

Some functions return Boolean values.

#### Checking Strings

```python
station_name = "ROBOT_LAB"
print(station_name.isupper())
print(station_name.islower())
```

### Boolean in Real Robot Logic (Example)

```python
battery_level = 65
obstacle_detected = False
robot_can_move = battery_level > 30 and not obstacle_detected
print(robot_can_move)
```

Output:
```
True
```

### Boolean Type Check

You can check the data type of a Boolean:

```python
is_connected = True
print(type(is_connected))
```

Output:
```
<class 'bool'>
```

### Boolean Values as Numbers (Important Concept)

In Python:

- True behaves like 1
- False behaves like 0

```python
print(True + True)
print(True * 5)
print(False + 10)
```

### Full Boolean Example (Complete Code)

```python
robot_active = True
emergency_stop = False
battery_level = 65
obstacle_detected = False
robot_name = "ATLAS"

print(robot_active)
print(emergency_stop)
print(battery_level > 50)
print(battery_level < 20)
print(robot_name == "ATLAS")
print(robot_name.isupper())

robot_can_move = robot_active and battery_level > 30 and not obstacle_detected

print(robot_can_move)
print(type(robot_can_move))

print(True + True)
print(False * 10)
```

### Common Beginner Mistakes

Using variable before assigning:

```python
print(speed)  # Error
```

Mixing types incorrectly:

```python
speed = "10"
speed + 5  # Error
```
![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/type_conversion_in_python_3.webp)


---

## Hands-On (Robot Operation Decision)

### Scenario

A factory uses an automated system to monitor whether an industrial robot is safe to operate. The system receives information about the robot's identity, the total number of robots in the factory, how many of them are currently broken, the robot's battery level, and whether an obstacle has been detected near the robot. Based on this information, the system must evaluate the robot's condition and decide if it can continue operating. A robot is considered safe only when its battery level is above 30, no obstacle is detected, fewer than half of the robots in the factory are broken, and the robot's name follows the factory rule of being written entirely in uppercase. The system should calculate the ratio of broken robots to total robots, determine whether the robot is allowed to operate using logical reasoning only, and then display the robot's information along with a clear message indicating whether operation is allowed or not.

---

### Task Requirements

#### Declare Your Own Variables

Create variables to represent:

- Robot name
- Total number of robots
- Number of broken robots
- Battery level
- Obstacle detection status

#### Calculation

- Calculate the **broken ratio** (broken ÷ total).

#### Decision Logic

The robot is allowed to operate **only if**:

- Battery level is greater than 30
- No obstacle is detected
- Broken robots are less than half of total robots
- Robot name is written in uppercase

Store the final decision in a Boolean variable called:

```python
robot_can_operate
```

#### Expected Output

Print:

- Robot name
- Broken ratio
- Battery level
- Obstacle status
- A final message:
  - "Robot is allowed to operate"
  - or
  - "Robot operation is NOT allowed"

---

## Python Data Structures -- Lists, Dictionaries, and Tuples

### Section Objective

By the end of this section, learners will be able to:

- Understand **why** data structures are needed
- Use **lists**, **tuples**, and **dictionaries**
- Choose the correct structure for a given robotics task
- Apply them in realistic robot scenarios (sensors, motors, states)

---

## Lists in Python

A **list** is a data structure that allows you to store **multiple values in one variable**. Lists are very flexible and can store numbers, strings, or even other lists.
Lists are created using **square brackets** `[]`, and items are separated by commas.


![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/python-list.jpg)


### Creating a List

Here is an example of a list that stores robot models used in a factory:

```python
factory_robots = ["ABB", "KUKA", "FANUC", "Omron"]
```

### Accessing List Items

Each item in a list has an **index**, starting from 0.

```python
print(factory_robots[0])  # first item
print(factory_robots[1])  # second item
```

### List Slicing

You can access a range of items using slicing:

```python
print(factory_robots[:3])  # first three items
```

### Adding Items to a List

You can add a new item to the end of a list using `append()`:

```python
factory_robots.append("UR")
print(factory_robots)
```

### Removing Items from a List

To remove a specific item:

```python
factory_robots.remove("UR")
print(factory_robots)
```

### Inserting Items at a Specific Position

You can insert an item at a specific index using `insert()`:

```python
factory_robots.insert(1, "UR")
print(factory_robots)
```

### Sorting a List

To sort the list alphabetically:

```python
factory_robots.sort()
print(factory_robots)
```

### Reversing a List

To reverse the order of items:

```python
factory_robots.reverse()
print(factory_robots)
```

### Length of a List

To find how many items are in a list:

```python
print(len(factory_robots))
```

### Extending a List

You can add another list to an existing list using `extend()`:

```python
extra_robots = ["Yaskawa", "Staubli"]
factory_robots.extend(extra_robots)
print(factory_robots)
```

### Removing the Last Item

To remove the last item in the list:

```python
factory_robots.pop()
print(factory_robots)
```

### Finding Items in a List

To get the index of an item:

```python
print(factory_robots.index("ABB"))
```

To count how many times an item appears:

```python
print(factory_robots.count("ABB"))
```

### Copying a List

To make a copy of a list:

```python
backup_robots = factory_robots.copy()
print(factory_robots)
print(backup_robots)
```

### Clearing a List

To remove all items from a list:

```python
factory_robots.clear()
print(factory_robots)
```

---

## 2D Lists (Nested Lists)

A **2D list** is a list that contains other lists.
It is commonly used to represent **tables, grids, or matrices**.

### Creating a 2D List

Here is an example of a **3 × 3 robot grid**:

```python
robot_grid = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 9]
]
```

### Accessing Elements in a 2D List

You access elements using **row index first**, then **column index**.

```python
print(robot_grid[0][0])  # first row, first column
print(robot_grid[1][2])  # second row, third column
```

### Modifying Elements in a 2D List

You can change a value by assigning a new one:

```python
robot_grid[1][2] = 10
print(robot_grid)
```

### Full Code (Lists + 2D Lists)

```python
# list of robot manufacturers
factory_robots = ["ABB", "KUKA", "FANUC", "Omron"]
print(factory_robots)

print(factory_robots[1])
print(factory_robots[:3])

factory_robots.append("UR")
print(factory_robots)

factory_robots.remove("UR")
print(factory_robots)

factory_robots.insert(1, "UR")
print(factory_robots)

factory_robots.sort()
print(factory_robots)

factory_robots.reverse()
print(factory_robots)

print(len(factory_robots))

extra_robots = ["Yaskawa", "Staubli"]
factory_robots.extend(extra_robots)
print(factory_robots)

factory_robots.pop()
print(factory_robots)

print(factory_robots.index("ABB"))
print(factory_robots.count("ABB"))

backup_robots = factory_robots.copy()
print(factory_robots)
print(backup_robots)

for robot in factory_robots:
    print(robot)

factory_robots.clear()
print(factory_robots)

# 2D list example
robot_grid = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 9]
]

print(robot_grid)
print(robot_grid[0][0])
print(robot_grid[1][2])

robot_grid[1][2] = 10
print(robot_grid)
```

---

## Hands-On (Factory Robot Layout Update)

A factory control system keeps track of the robot manufacturers used on the production floor and stores robot positions in a grid that represents different production lines. During a system update, a new robot manufacturer is added, one manufacturer is removed, and the list needs to be reorganized to reflect the current setup. At the same time, the factory layout grid must be checked to identify specific robot positions, and one robot in the grid needs to be replaced due to maintenance. After the update, the system should display the current list of manufacturers, the total number of manufacturers, selected manufacturers by position, and the updated factory layout grid.

---

## Tuples in Python

A **tuple** is another data type that allows you to store **multiple values in a single variable**.
Tuples are very similar to lists, but there is one **important difference**:

**Tuples are immutable**, which means once a tuple is created, its values **cannot be changed**.

Tuples are created using **parentheses** `()` instead of square brackets.


![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/python-tuple.jpg)



### Creating a Tuple

Here is an example of a tuple that stores the names of control engineers in a robotics factory:

```python
control_engineers = ("Hannah", "Jim", "Bella", "John")
```

### Accessing Tuple Values

You can access tuple elements using **indexing**, just like lists.

```python
print(control_engineers[1])  # prints the second engineer
```

Remember:

- Indexing starts from 0
- `control_engineers[0]` → first value

### Slicing a Tuple

You can also slice a tuple to get multiple values:

```python
print(control_engineers[:2])  # prints the first two engineers
```

### Unpacking a Tuple

Tuples can be unpacked into multiple variables.
Each variable gets one value from the tuple.

```python
eng1, eng2, eng3, eng4 = control_engineers
print(eng1, eng2, eng3, eng4)
```

You can access any unpacked value directly:

```python
print(eng3)
```

### Immutability of Tuples

Tuples **cannot be modified** after creation.
Trying to change a value will cause an error.

```python
# This will cause an error
# control_engineers[1] = "Bob"
```

This is why tuples are often used for **fixed data** that should not change.

### Tuples with Different Data Types

A tuple can store different data types together:

```python
robot_info = ("XR-21", 85, True)
```

### List of Tuples

You can create a **list of tuples**, which is very common in robotics and data processing.

Example: pairing engineers for robot calibration:

```python
engineer_pairs = [
    (control_engineers[0], control_engineers[3]),
    (control_engineers[1], control_engineers[2])
]
```

### Accessing Tuples Inside a List

```python
print(engineer_pairs)
print(engineer_pairs[0])
print(engineer_pairs[0][1])
```

### Full Code (Tuples)

```python
# this section is about tuples
control_engineers = ("Hannah", "Jim", "Bella", "John")
print(control_engineers)

# print the second value in the tuple
print(control_engineers[1])

# slicing the tuple
print(control_engineers[:2])

# unpacking the tuple
eng1, eng2, eng3, eng4 = control_engineers
print(eng1, eng2, eng3, eng4)
print(eng3)

# tuples are immutable
# control_engineers[1] = "Bob"

# list of tuples (engineer pairs)
engineer_pairs = [
    (control_engineers[0], control_engineers[3]),
    (control_engineers[1], control_engineers[2])
]

print(engineer_pairs)
print(engineer_pairs[0])
print(engineer_pairs[0][1])
```


---

## Dictionaries in Python

Another important data type in Python is the **dictionary**.

A **dictionary** stores data in **key–value pairs**.
Instead of accessing values by index (like lists and tuples), we access them using a **key**.

### Key Concepts

- Each **key** is unique (cannot be repeated)
- Keys are **case-sensitive**
- Keys must be **immutable** (strings, numbers, tuples)
- Values can be **any data type**
- Dictionaries are created using **curly braces** `{}`


![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/dictionaries-in-python.jpg)


### Creating a Dictionary

Example:
A robot arm is detecting objects and storing their colors.

```python
object_colors = {
    "ball": "red",
    "cube": "green",
    "flower": "pink",
    "pyramid": "blue"
}
```

Here:

- `"ball"`, `"cube"`, `"flower"` are **keys**
- `"red"`, `"green"`, `"pink"` are **values**

### Accessing Values Using Keys

```python
print("The ball is", object_colors["ball"])
print("The cube is", object_colors["cube"])
```

You access the value by writing the **dictionary name** followed by the **key** inside square brackets.

### Using the get() Method

The `get()` method is a safer way to access dictionary values.

```python
print(object_colors.get("flower"))
print(object_colors.get("box", "no such object"))
```

If the key exists, `get()` returns its value.
If it does not exist, it returns the **default value** you provide.

### Modifying a Dictionary

#### Adding a New Key–Value Pair

```python
object_colors["cylinder"] = "yellow"
print(object_colors)
```

#### Updating an Existing Value

```python
object_colors["ball"] = "orange"
print(object_colors)
```

### Dictionary with Numeric Values (Robot Example)

Example: storing robot joint default angles.

```python
joint_defaults = {
    "joint1": 0,
    "joint2": 45,
    "joint3": -30
}
```

Accessing values:

```python
print(joint_defaults["joint2"])
```

### Nested Dictionaries (Dictionary Inside Dictionary)

Dictionaries can store **other dictionaries** as values.

Example: robot arm joint configuration.

```python
arm_config = {
    "joint1": {
        "min": -90,
        "max": 90,
        "default": 0
    },
    "joint2": {
        "min": -180,
        "max": 180,
        "default": 0
    },
    "joint3": {
        "min": -90,
        "max": 90,
        "default": 0
    }
}
```

### Accessing Nested Dictionary Values

```python
print(arm_config["joint1"])
print(arm_config["joint2"]["max"])
print(arm_config["joint3"]["min"])
```

You first access the **outer key**, then the **inner key**.

### Full Code (Dictionaries)

```python
# dictionary mapping objects to colors
object_colors = {
    "ball": "red",
    "cube": "green",
    "flower": "pink",
    "pyramid": "blue"
}

print("The ball is", object_colors["ball"])
print("The cube is", object_colors["cube"])
print(object_colors.get("box", "no such object"))

# adding and updating values
object_colors["cylinder"] = "yellow"
object_colors["ball"] = "orange"
print(object_colors)

# dictionary for robot arm configuration
arm_config = {
    "joint1": {
        "min": -90,
        "max": 90,
        "default": 0
    },
    "joint2": {
        "min": -180,
        "max": 180,
        "default": 0
    },
    "joint3": {
        "min": -90,
        "max": 90,
        "default": 0
    }
}

print(arm_config["joint1"])
print(arm_config["joint2"]["max"])
print(arm_config["joint3"]["min"])
```

### Choosing the Right Structure

| Task | Best Choice |
|------|-------------|
| Sensor readings | List |
| Fixed coordinates | Tuple |
| Robot configuration | Dictionary |
| Motor names | List |
| Robot state | Dictionary |

### Common Beginner Mistakes

- Mixing list & dictionary syntax
- Forgetting commas
- Modifying tuples
- Using numbers instead of keys in dictionaries

---

## Python Conditions -- Teaching the Robot to Decide

### Section Objective

By the end of this section, learners will be able to:

- Understand how decision-making works in Python
- Use `if`, `elif`, and `else` correctly
- Combine conditions with variables, loops, and data structures
- Implement simple robot behaviors based on conditions

### What Is a Condition?

In Python, **conditional statements** allow your program to make decisions.
They let the code **choose different actions** depending on whether a condition is True or False.

The most common conditional statement is the **if statement**.

### Basic Syntax

```python
if condition:
    # code executed if the condition is True
elif another_condition:
    # code executed if the previous condition was False
else:
    # code executed if all conditions are False
```

- The condition must evaluate to **True or False**
- Only **one block** will be executed
- `elif` and `else` are optional

### Comparing Values

Conditions are usually written using **comparison operators**:

- `==` equal
- `!=` not equal
- `>` greater than
- `<` less than
- `>=` greater than or equal
- `<=` less than or equal

They can be combined using **logical operators**:

- `and`
- `or`
- `not`

### Example 1: Robot Arm Controlled by a Joystick

A robot arm has **two joints** and is controlled using a joystick:

- Right → first joint rotates in positive direction
- Left → first joint rotates in negative direction
- Up → second joint rotates in positive direction
- Down → second joint rotates in negative direction
- Neutral → no movement

```python
joystick_input = "up"

if joystick_input == "right":
    print("Moving the first joint in positive direction")
elif joystick_input == "left":
    print("Moving the first joint in negative direction")
elif joystick_input == "up":
    print("Moving the second joint in positive direction")
elif joystick_input == "down":
    print("Moving the second joint in negative direction")
else:
    print("Joystick is in neutral position, no movement")
```

In a real robot, instead of print, you would send commands to the motors.

### Example 2: Mobile Robot Obstacle Detection

A mobile robot has **two proximity sensors**.
If **any sensor** detects an obstacle closer than a safe distance, the robot must stop.

```python
sensor1_distance = 1.5  # meters
sensor2_distance = 1.2  # meters
safe_distance = 1.0     # meters

if sensor1_distance < safe_distance or sensor2_distance < safe_distance:
    print("Warning: Obstacle detected! Stopping robot.")
else:
    print("Path is clear. Moving forward.")
```

Here:

- The condition uses the `or` operator
- Only one sensor needs to detect danger to stop the robot

### Why if Statements Are Important in Robotics

Conditional statements allow robots to:

- React to sensor data
- Make movement decisions
- Avoid obstacles
- Choose different behaviors
- Handle safety conditions

Without `if` statements, robots would not be able to **adapt** to their environment.

### Full Code (Conditional Statements)

```python
# joystick control example
joystick_input = "up"

if joystick_input == "right":
    print("Moving the first joint in positive direction")
elif joystick_input == "left":
    print("Moving the first joint in negative direction")
elif joystick_input == "up":
    print("Moving the second joint in positive direction")
elif joystick_input == "down":
    print("Moving the second joint in negative direction")
else:
    print("Joystick is in neutral position, no movement")

# obstacle detection example
sensor1_distance = 1.5
sensor2_distance = 1.2
safe_distance = 1.0

if sensor1_distance < safe_distance or sensor2_distance < safe_distance:
    print("Warning: Obstacle detected! Stopping robot.")
else:
    print("Path is clear. Moving forward.")
```

---

## Python Loops -- Teaching the Robot to Repeat Actions

### Section Objective

By the end of this section, learners will be able to:

- Understand why loops are essential in programming and robotics
- Use `for`, `for-else`, and `while` loops correctly
- Work with the `range()` function
- Apply loops to real robot behaviors (movement, sensing, monitoring)

### What Is a Loop?


A **loop** is a structure that repeats a block of code multiple times.

---

## While Loops in Python

A **while loop** is used to repeatedly execute a block of code **as long as a condition remains true**.

### Basic Syntax

```python
while condition:
    # code to be executed repeatedly
```

How it works:

1. The condition is checked.
2. If it is True, the code inside the loop runs.
3. After execution, the condition is checked again.
4. The loop stops when the condition becomes False.

⚠️ If the condition never becomes false, the loop becomes an **infinite loop**.

### Example: Robot Moving Until an Obstacle Is Detected

A robot moves forward step by step until its sensor detects an obstacle.

```python
# robot initial position
position = 0

# simulated obstacle flag
obstacle_detected = False

while not obstacle_detected:
    print("Robot moving forward...")
    position += 1
    if position == 5:
        obstacle_detected = True

print("Obstacle detected! Robot stopped.")
print("Final position:", position)
```

This example shows how the loop continues **until a condition changes**.

---

## For Loops in Python

A **for loop** is used to iterate over a **sequence** (string, list, range, etc.).

![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/for-loop-in-python%20(1).png)

### Basic Syntax

```python
for element in iterable:
    # code executed for each element
```

The loop stops automatically when all elements are processed.

### For Loop with a String

```python
for letter in "Robotics":
    print(letter)
```

Each character in the string is printed one by one.

### For Loop with a List

```python
robotics_engineers = ["Tim", "Hannah", "John", "Bella"]

for engineer in robotics_engineers:
    print(engineer)
```

Each iteration gives one engineer name.

### For Loop with Indexes

Sometimes you need the **index and the value**.

```python
for index in range(len(robotics_engineers)):
    print(index)
    print(robotics_engineers[index])
```

Here:

- `range(len(...))` generates indexes
- Indexing is used to access list elements

---

## The range() Function

The `range()` function generates a sequence of numbers.

### Simple Range Loop

```python
for number in range(5):
    print(number)
```

Output:
```
0 1 2 3 4
```

### Range with Start and End

```python
for number in range(3, 10):
    print(number)
```
![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/for-loop-range.png)

Numbers start from 3 and stop at 9.

### Range with Step

```python
for number in range(0, 10, 2):
    print(number)
```

This counts by **2**.

---

## for-else Loop

A **for-else loop** executes the else block **only if the loop finishes normally** (without `break`).

### Example: Searching for a Robot ID

```python
robot_ids = [101, 102, 103, 104]
search_id = 105

for robot in robot_ids:
    if robot == search_id:
        print("Robot found!")
        break
else:
    print("Robot not found in the system.")
```

Here:

- If `break` happens → `else` is skipped
- If no `break` → `else` runs

---

## Nested Loops

A **nested loop** is a loop inside another loop.
They are commonly used with **2D lists (grids)**.

### Example: Iterating Over a Robot Grid

```python
robot_grid = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 9]
]

for row in robot_grid:
    for element in row:
        print(element)
```

- Outer loop → rows
- Inner loop → columns

---

## Full Code (While, For, Range, For-Else, Nested)

```python
# while loop example
position = 0
obstacle_detected = False

while not obstacle_detected:
    print("Robot moving forward...")
    position += 1
    if position == 5:
        obstacle_detected = True

print("Obstacle detected! Robot stopped.")
print("Final position:", position)

# for loop with string
for letter in "Robotics":
    print(letter)

# for loop with list
robotics_engineers = ["Tim", "Hannah", "John", "Bella"]

for engineer in robotics_engineers:
    print(engineer)

# for loop with indexes
for index in range(len(robotics_engineers)):
    print(index, robotics_engineers[index])

# range loop
for number in range(3, 10):
    print(number)

# for-else loop
robot_ids = [101, 102, 103]
search_id = 105

for robot in robot_ids:
    if robot == search_id:
        print("Robot found!")
        break
else:
    print("Robot not found.")

# nested loops with 2D list
matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

for row in matrix:
    for element in row:
        print(element)
```

### Infinite Loop Warning

Dangerous code:

```python
while True:
    print("Robot moving forever")
```

Safe version:

```python
while True:
    command = input("Enter command: ")
    if command == "stop":
        break
```

Always ensure the condition **can change** or you use `break`

---

## Loop Control Keywords

### 🛑 break

Stops the loop immediately.

### ⏭️ continue

Skips current iteration.

```python
for i in range(5):
    if i == 2:
        continue
    print(i)
```

### Common Beginner Mistakes

- Forgetting indentation
- Infinite while loops
- Misunderstanding `range()`
- Expecting for-else to work like if-else

---

## Robotics Mapping Summary

| Robot Task | Loop Used |
|------------|-----------|
| Sensor monitoring | while |
| Fixed movements | for |
| Scanning area | for-else |
| Gradual speed change | range() |

---

## Python Functions -- Teaching the Robot to Reuse Intelligence

### Section Objective

By the end of this section, learners will be able to:

- Understand why functions are needed
- Define and call functions in Python
- Use parameters and return values
- Apply functions to real robotics behaviors
- Write clean, reusable code

### What Is a Function?

A function is a block of code that performs a specific task. It can be used to group related code together, make code more reusable, and make code easier to read and understand.

The `return` statement is used to exit a function and return a value to the caller. The value that is returned can be any Python object, such as a number, string, list, or dictionary. After the `return`, no code will be executed because when Python sees the return keyword it will break out of the code.

![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/Python-Function-Syntax.png)

Functions can be passed data in two ways:

- **Arguments**. Arguments are passed to a function when it is called. They can be any type of value, such as numbers, strings, lists, or dictionaries.
- **Parameters**. Parameters are variables that are declared in the function definition. They are used to store the values of the arguments that are passed to the function.

In robotics software, **functions are the backbone of the system**.
Every robot action—moving, sensing, checking safety, making decisions—is usually implemented as a function.

![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/parameter%26arguments.png)

### Defining a Function

```python
def function_name(parameters):
    # code block
```

- `def` tells Python we are defining a function
- `parameters` are inputs to the function
- The function body must be indented

### Function That Performs an Action (No Return)

Example: commanding a robot to move.

```python
def move_robot(direction, distance):
    print("Moving robot", direction)
    print("Distance:", distance)

move_robot("forward", 3)
move_robot("backward", 1)
```

This type of function **performs an action**, but does not return a value.

### Function That Returns Information

Many robot functions **calculate or evaluate** something and return the result.

Example: checking battery safety.

```python
def battery_is_safe(level):
    return level > 30

battery_level = 45
print(battery_is_safe(battery_level))
```

The function returns a **Boolean**, which can be reused in decision logic.

### Functions + Conditions (Decision Functions)

Robots often need functions that **make decisions**.

```python
def can_robot_move(battery, obstacle_detected):
    if battery > 30 and not obstacle_detected:
        return True
    return False

print(can_robot_move(50, False))
print(can_robot_move(20, True))
```

### Functions Returning Calculated Values

Example: computing traveled distance.

```python
def calculate_distance(speed, time):
    return speed * time

distance = calculate_distance(2, 5)
print("Distance traveled:", distance)
```

### Functions Returning Strings (Status Messages)

Functions can also return **formatted text**.

```python
def robot_status(name, active):
    if active:
        return "Robot " + name + " is ACTIVE"
    else:
        return "Robot " + name + " is INACTIVE"

print(robot_status("XR-21", True))
```

### Using Default Parameter Values

Default parameters make functions more flexible.

```python
def move_robot(direction="forward", speed=1):
    print("Direction:", direction)
    print("Speed:", speed)

move_robot()
move_robot("left", 3)
```

### Functions Calling Other Functions

Real robot systems are **layers of functions**.

```python
def read_sensor():
    return 0.8

def obstacle_detected():
    return read_sensor() < 1.0

print(obstacle_detected())
```

### Example: Complete Robot Control Logic Using Functions

```python
def battery_is_safe(level):
    return level > 30

def obstacle_detected(sensor_distance):
    return sensor_distance < 1.0

def can_robot_operate(battery, sensor_distance):
    return battery_is_safe(battery) and not obstacle_detected(sensor_distance)

print(can_robot_operate(50, 1.5))
print(can_robot_operate(20, 0.5))
```

### Why Functions Matter in Robotics

Functions allow you to:

- separate sensing, logic, and action
- reuse robot behaviors
- test individual components
- scale programs safely
- transition naturally to **classes and ROS nodes**

### Full Code (Powerful Functions Example)

```python
def battery_is_safe(level):
    return level > 30

def obstacle_detected(distance):
    return distance < 1.0

def calculate_distance(speed, time):
    return speed * time

def move_robot(direction, distance):
    print("Moving", direction, "for", distance, "units")

def can_robot_operate(battery, sensor_distance):
    return battery_is_safe(battery) and not obstacle_detected(sensor_distance)

move_robot("forward", 3)

distance = calculate_distance(2, 4)
print("Distance traveled:", distance)

print("Battery safe:", battery_is_safe(40))
print("Can operate:", can_robot_operate(50, 1.5))
print("Can operate:", can_robot_operate(20, 0.5))
```

---

## Hands On (Factory Robot Controller)

A factory robot has a fixed identity and operates inside a square workspace.
The robot moves forward in equal steps while it is safe to operate.

Safety is determined by a function that checks the robot's current battery level and obstacle status and returns whether operation is allowed.
Movement is handled by a function that receives the current position and movement speed and returns the updated position.

The robot's internal state (battery, speed, obstacle status) is stored as structured data and is updated on every step.
Each movement step must be recorded in the workspace grid.

The robot continues operating until the safety check fails.
When the robot stops, the program prints the robot's identity, final position, final battery level, and the final workspace grid.

---

## Classes and Objects in Python

In the previous sections, we worked with built-in data types such as numbers, strings, booleans, and data structures like lists and dictionaries.
However, many real-world entities—such as **robots, sensors, computers, or people**—cannot be represented naturally using only these data types.

To model such real-world entities, Python allows us to create **our own data types** using **classes and objects**.

### What Is a Class?

A **class** is a user-defined blueprint that describes:

- what data an object should store (**attributes**)
- what actions an object can perform (**methods**)

In other words, a class defines **a new data type**.

### What Is an Object?

An **object** is an instance of a class.
When we create an object from a class, Python allocates memory for it and assigns values to its attributes.

Multiple objects can be created from the same class, and:

- they share the same methods
- each object has its **own attribute values**

### Defining a Class

In Python, a class is defined using the `class` keyword.

Let's define a simple **robot arm class**.

Assumptions:

- The robot arm has **one joint** (1 degree of freedom)
- The joint position is represented by an angle
- The robot can move and display its information


![Diagram](https://github.com/cyber-ai-dep/ROS2-Basics/blob/main/assets/images/fundamentals/class_method%20(1).png)


### Example: Robot Arm Class

```python
class RobotArm:
    def __init__(self, name, length, weight, color):
        self.name = name
        self.length = length
        self.weight = weight
        self.color = color
        self.position = 0
```

#### Explanation

- `__init__` is called the **constructor**
- It runs automatically when an object is created
- `self` refers to the current object
- Attributes like `name`, `length`, and `position` belong to each object

### Adding Methods to a Class

Methods are functions defined inside a class that describe the object's behavior.

```python
class RobotArm:
    def __init__(self, name, length, weight, color):
        self.name = name
        self.length = length
        self.weight = weight
        self.color = color
        self.position = 0
    
    def move(self, angle):
        self.position += angle
    
    def display_info(self):
        print(f"Name: {self.name}")
        print(f"Length: {self.length} cm")
        print(f"Weight: {self.weight} kg")
        print(f"Color: {self.color}")
        print(f"Position: {self.position} degrees")
```

### Creating an Object

Once the class is defined, we can create objects from it.

```python
arm1 = RobotArm("Armrob", 50, 10, "Black")
```

Here:

- `arm1` is an object
- `RobotArm(...)` calls the constructor
- The object now has its own attributes and methods

### Using Object Methods

We access attributes and methods using **dot notation**.

```python
arm1.move(45)
arm1.display_info()
```

The method updates the internal state (position) of the object.

### Updating Object State

Objects keep track of their own state.

```python
arm1.move(-15)
arm1.display_info()
```

Each call to `move()` changes the robot arm's position.

### Accessing Attributes Directly

You can also access object attributes directly.

```python
print(arm1.name)
print(arm1.position)
```

### Multiple Objects from the Same Class

```python
arm2 = RobotArm("ArmX", 60, 12, "Red")
arm2.move(30)
arm2.display_info()
```

- `arm1` and `arm2` are different objects
- They share the same class
- Each has its own data

### Full Code (Classes and Objects)

```python
class RobotArm:
    def __init__(self, name, length, weight, color):
        self.name = name
        self.length = length
        self.weight = weight
        self.color = color
        self.position = 0
    
    def move(self, angle):
        self.position += angle
    
    def display_info(self):
        print(f"Name: {self.name}")
        print(f"Length: {self.length} cm")
        print(f"Weight: {self.weight} kg")
        print(f"Color: {self.color}")
        print(f"Position: {self.position} degrees")

arm1 = RobotArm("Armrob", 50, 10, "Black")
arm1.move(45)
arm1.display_info()

arm1.move(-15)
arm1.display_info()

arm2 = RobotArm("ArmX", 60, 12, "Red")
arm2.move(30)
arm2.display_info()
```

### Why Classes Are Important in Robotics

Classes allow you to:

- model real robot components
- group data and behavior together
- create multiple robots easily
- organize complex systems
- prepare for advanced topics like control systems and ROS

### Common Beginner Mistakes

- **Forgetting** `self`
- Calling methods without parentheses
- Confusing class vs object
- Writing logic outside the class

---

## Python Modules & Imports -- Teaching the Robot to Use External Skills

### Section Objective

By the end of this section, learners will be able to:

- Understand what a **module** is
- Import built-in Python modules
- Import specific functions or classes
- Create and import their **own modules**
- Organize robotics code in a clean way

### What Is a Module?

#### Simple Definition

A **module** is a Python file (.py) that contains:

- Variables
- Functions
- Classes

Example:
```
robot_utils.py
```

### Importing Built-in Modules

Python comes with many ready-made modules.

#### Example: time Module

```python
import time

print("Robot starting...")
time.sleep(2)
print("Robot moving")
```

Robot logic:
Pause execution to simulate real movement timing.

#### Example: math Module

```python
import math

distance = 5
area = math.pi * distance ** 2
print(area)
```

### Different Ways to Import

#### 1. Import Entire Module

```python
import math
print(math.sqrt(16))
```

#### 2. Import Specific Functions

```python
from math import sqrt
print(sqrt(16))
```

#### 3. Import with Alias

```python
import time as t
t.sleep(1)
```

Aliases make code:

- Shorter
- Cleaner
- Easier to read

### Importing Your Own Module

#### Project Structure

```
robot_project/
│
├── main.py
├── robot.py
```

#### robot.py

```python
class Robot:
    def __init__(self, name):
        self.name = name
    
    def move(self):
        print(self.name, "is moving")
```

#### main.py

```python
from robot import Robot

robot1 = Robot("MechDog")
robot1.move()
```

What happens:

- `robot.py` defines the robot
- `main.py` **uses** the robot

### Importing Functions from a Module

#### sensors.py

```python
def read_distance():
    return 35
```

#### main.py

```python
from sensors import read_distance

distance = read_distance()
print("Distance:", distance)
```

### Why Imports Matter in Robotics

Without imports:

- One huge file
- Hard to debug
- Hard to scale

With imports:

- Clean separation
- Reusable logic
- Team-friendly code

---

## Mini Scenario -- Modular Robot System

#### movement.py

```python
def move_forward():
    print("Moving forward")
```

#### safety.py

```python
def check_battery(level):
    return level > 20
```

#### main.py

```python
from movement import move_forward
from safety import check_battery

battery = 50

if check_battery(battery):
    move_forward()
else:
    print("Low battery")
```

This shows:

- Clear responsibilities
- Easy upgrades
- **Real robotics structure**

### How This Connects to ROS 2 (Conceptual)

- Each **node** = module
- Each **behavior** = function or class
- Imports = node communication logic


```


















