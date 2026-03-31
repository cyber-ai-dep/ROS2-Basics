# ------------------------------------------
# -- Object Oriented Programming => Intro --
# ------------------------------------------
# [1] Python Support Object Oriented Programming
# [2] OOP Is A Paradigm Or Coding Style
#     OOP Paradigm =>
#       Means Structuring Robot Program So The Methods[Functions] and Attributes[Data]
#       Are Bundled Into Robot Objects
# [3] Methods => Act As Functions That Use The Information Of The Robot Object
#     Example: move(), stop(), read_sensor(), publish_topic()
# [4] Python Is Multi-Paradigm Programming Language [Procedural, OOP, Functional]
#     - Procedural => Structure Robot Task Like Steps
#       Example: Start Motor -> Move Forward -> Stop
#     - Functional => Robot Behaviors Built On Mathematical Functions
#       Example: calculate_speed(distance), compute_angle()
# [5] OOP Allow You To Organize Your Robot Code
#     Separate Sensors, Motors, Controllers Into Clean Classes
#     Make Code Readable, Scalable, and Reusable
# [6] Everything in Python is Object
#     Even Robot Nodes, Messages, Publishers, and Subscribers Are Objects
# [7] If Robot Is Object
#     - Attributes => Name, ID, Battery_Level, Position, Speed, Sensor_Data
#     - Methods[Behaviors] => Move, Stop, Rotate, Charge, Read_Sensors, Send_Data
# [8] If Sensor Is Object
#     - Attributes => Type, Range, Resolution, Port_Number
#     - Methods[Behaviors] => Measure, Calibrate, Publish_Data
# [9] Class Is The Template For Creating Objects [Object Constructor | Blueprint]
#     - Class Robot Can Create Many Robot Objects
#       Example: Robot1, Robot2, FactoryRobot, DeliveryRobot
# ---------------------------------------------


# ----------------------------------------------------------
# -- Object Oriented Programming => Class Syntax and Info --
# ----------------------------------------------------------
# [01] Class is The Blueprint Or Construtor Of The Robot Object
# [02] Class Instantiate Means Create Instance of A Robot From The Class
# [03] Instance => Robot Object Created From Class And Have Its Methods and Attributes
# [04] Class Defined With Keyword class
# [05] Class Name Written With PascalCase [UpperCamelCase] Style
#     Example: MobileRobot, RobotArm, DistanceSensor
# [06] Class May Contains Robot Methods and Robot Attributes
#     Example: move(), stop(), battery_level, position
# [07] When Creating Robot Object Python Look For The Built In __init__ Method
# [08] __init__ Method Called Every Time You Create Robot From Class
# [09] __init__ Method Is Initialize The Robot Data
#     Example: Name, ID, Speed, Initial Position
# [10] Any Method With Two Underscore in The Start and End Called Dunder or Magic Method
#     Example: __init__, __str__
# [11] self Refer To The Current Robot Instance Created From The Class And Must Be First Param
# [12] self Can Be Named Anything
# -------------------------------------------------------------------


# Syntax
# class Name:
#     Constructor => Do Instantiation [ Create Instance From A Class ]
#     Each Instance Is Separate Object
#     def __init__(self, other_data)
#         Body Of Function




# ------------------------------------------
# -- Object Oriented Programming => Intro --
# ------------------------------------------

class Robot:

  def __init__(self):
    print("A New Robot Has Been Activated 🤖")


robot_one = Robot()
robot_two = Robot()
robot_three = Robot()

print(robot_one.__class__)





# --------------------------------------------------------------------
# -- Object Oriented Programming => Instance Attributes and Methods --
# --------------------------------------------------------------------
# Self: Point To Instance Created From Class
# Instance Attributes: Instance Attributes Defined Inside The Constructor
# -----------------------------------------------------------------------
# Instance Methods: Take Self Parameter Which Point To Instance Created From Class
# Instance Methods Can Have More Than One Parameter Like Any Function
# Instance Methods Can Freely Access Attributes And Methods On The Same Object
# Instance Methods Can Access The Class Itself
# -----------------------------------------------------------


# Instance Attributes (Robot Identity)
class Robot:

  def __init__(self, name, robot_type, battery_level):
    self.name = name
    self.robot_type = robot_type
    self.battery_level = battery_level
    # Now we give each robot specific data.

robot_one = Robot("Atlas", "Humanoid", 90)
robot_two = Robot("Rover", "Wheeled", 75)
robot_three = Robot("Nano", "Drone", 60)

print(robot_one.name, robot_one.robot_type, robot_one.battery_level)
print(robot_two.name)
print(robot_three.name)


# Instance Methods (Robot Behaviors)
class Robot:

  def __init__(self, name, robot_type, battery_level):
    self.name = name
    self.robot_type = robot_type
    self.battery_level = battery_level

  def introduce(self):
    return f"I am {self.name}, a {self.robot_type} robot."

  def check_battery(self):
    return f"{self.name} Battery Level: {self.battery_level}%"

  def perform_task(self, task):
    return f"{self.name} is performing task: {task}"
  
  # Now we add robot behaviors (methods).


robot_one = Robot("Atlas", "Humanoid", 90)
robot_two = Robot("Rover", "Wheeled", 75)

print(robot_one.introduce())
print(robot_two.perform_task("Inspect Area"))




# -----------------------------------------------------
# -- Object Oriented Programming => Class Attributes --
# -----------------------------------------------------
# Class Attributes: Attributes Defined Outside The Constructor


# Class Attributes (Shared Among All Robots)

class Robot:

  restricted_names = ["Error", "Null", "Crash"]  
  total_robots = 0  # Now we will track total robots in the system.

  def __init__(self, name, robot_type, battery_level):
    self.name = name
    self.robot_type = robot_type
    self.battery_level = battery_level

    Robot.total_robots += 1

  def introduce(self):
    if self.name in Robot.restricted_names:
      raise ValueError("Robot Name Not Allowed ⚠️")
    return f"I am {self.name}, a {self.robot_type} robot."

  def shutdown(self):
    Robot.total_robots -= 1
    return f"{self.name} has been powered off."


print(Robot.total_robots)

robot_one = Robot("Atlas", "Humanoid", 90)
robot_two = Robot("Rover", "Wheeled", 75)
robot_three = Robot("Nano", "Drone", 60)

print(Robot.total_robots)

print(robot_three.shutdown())
print(Robot.total_robots)


# -------------------------------------------------------------------
# -- Object Oriented Programming => Class Methods & Static Methods --
# -------------------------------------------------------------------
# Class Methods:
# - Marked With @classmethod Decorator To Flag It As Class Method
# - It Take Cls Parameter Not Self To Point To The Class not The Instance
# - It Doesn't Require Creation of a Class Instance
# - Used When You Want To Do Something With The Class Itself
# Static Methods:
# - It Takes No Parameters
# - Its Bound To The Class Not Instance
# - Used When Doing Something Doesnt Have Access To Object Or Class But Related To Class
# -----------------------------------------------------------

# Class Methods & Static Methods

class Robot:

  restricted_names = ["Error", "Null", "Crash"]
  total_robots = 0

  @classmethod   
  def show_robot_count(cls):
    print(f"There Are {cls.total_robots} Robots In The System 🤖")

  @staticmethod
  def system_message():
    print("Robotics Control System Online 🚀")

  def __init__(self, name, robot_type, battery_level):
    self.name = name
    self.robot_type = robot_type
    self.battery_level = battery_level
    Robot.total_robots += 1

  def introduce(self):
    if self.name in Robot.restricted_names:
      raise ValueError("Robot Name Not Allowed ⚠️")
    return f"I am {self.name}, a {self.robot_type} robot."

  def shutdown(self):
    Robot.total_robots -= 1
    return f"{self.name} has been powered off."


print(Robot.total_robots)

robot_one = Robot("Atlas", "Humanoid", 90)
robot_two = Robot("Rover", "Wheeled", 75)

Robot.show_robot_count()

Robot.system_message()

print(robot_one.introduce())
print(Robot.introduce(robot_one))




