# ---------------------------------
# -- Modules => Built In Modules --
# ---------------------------------
# [1] Module is A File Contain A Set Of Functions
# [2] You Can Import Module in Your App To Help You
# [3] You Can Import Multiple Modules
# [4] You Can Create Your Own Modules
# [5] Modules Saves Your Time
# --------------------------------------------------

# Import built-in module
# import random

# print(random) 


# # # Show All Functions Inside Module
# print(dir(random))


# # Simulate random sensor noise
# sensor_value = 50
# noise = random.uniform(-2, 2)

# print(f"Original Sensor Value: {sensor_value}")
# print(f"Sensor With Noise: {sensor_value + noise}")


# # Random movement direction
# direction = random.choice(["LEFT", "RIGHT", "FORWARD", "BACKWARD"])
# print(f"Robot Moves: {direction}") 



# # Import One Or Two Functions From Module

# from random import randint 
# print(f"Print Random Integer {randint(100, 900)}")

# import random as rn
# print(rn.random())