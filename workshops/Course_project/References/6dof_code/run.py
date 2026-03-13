#!/usr/bin/env python3
import rclpy
from robot_movement import RobotMover


def main():
    rclpy.init()
    robot = RobotMover()

    # Always start from home
    robot.move_to('home_pos')
    robot.wait_until_done()

    # ─────────────────────────────────────
    #  Add your movement sequence below:
    # ─────────────────────────────────────
    # robot.move_to('position_1')
    # robot.wait_until_done()
    # robot.move_to('postposition_1')
    # robot.wait_until_done()

    # Return home
    robot.move_to('home_pos')
    robot.wait_until_done()

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()