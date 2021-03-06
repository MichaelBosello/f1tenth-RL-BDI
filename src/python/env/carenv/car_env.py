from threading import Thread
import numpy as np
import os
import random
import time

import rospy

from carenv.car.car_control import Drive
from carenv.car.safety_control import SafetyControl
from carenv.car.sensors import Sensors
from carenv.car.position import Position
from carenv.car.target import Target

MAX_STOP = 3

# you can set the reward according to the action performed or according to the linear velocity of the car
USE_VELOCITY_AS_REWARD = False
ADD_LIDAR_DISTANCE_REWARD = False
LIDAR_DISTANCE_WEIGHT = 0.1

# 0.55 real car 1/6 speed --- 0.46 simulator 1/3 speed
VELOCITY_NORMALIZATION = 0.55 # normalize the velocity between 0 and 1 (e.g. max velocity = 1.8 => 1.8*0.55 =~ 1)
REWARD_SCALING = 0.09 # scale the velocity rewards between [0, REWARD_SCALING]. I.e. at max velocity the reward is REWARD_SCALING

class CarEnv:
    @staticmethod
    def run_node():
        rospy.init_node('rl_driver')

    def __init__(self, args={}):
        self.is_simulator = args.get('simulator', 'false') == 'true'
        self.sensors = Sensors(is_simulator=self.is_simulator)
        self.control = Drive(self.sensors, is_simulator=self.is_simulator)
        self.safety_control = SafetyControl(self.control, self.sensors, is_simulator=self.is_simulator)
        self.target = Target(is_simulator=self.is_simulator)
        self.position = Position(self.control, is_simulator=self.is_simulator)
        time.sleep(4)

        # available actions
        self.action_set = [0, 1, 2]

        self.game_number = 0
        self.step_number = 0
        self.is_terminal = False
        self.stuck_count = 0

        def reset_handler():
            while True:
                user_input = input()
                if user_input == 'r':
                    print("Resetting simulator position...")
                    self.control.reset_simulator()

        process = Thread(target=reset_handler)
        process.daemon = True
        process.start()

        self.target.new_target()
        self.reset_game()

    def step(self, action):
        if action == -1:
            self.target.new_target()
            self.position.reset_to_pos("")
            self.reset_game()
            return self.reward, self.state, self.is_terminal

        if action <= -2:
            if action == -2:
                self.position.reset_to_pos("A")
            if action == -3:
                self.position.reset_to_pos("B")
            if action == -4:
                self.position.reset_to_pos("C")
            if action == -5:
                self.position.reset_to_pos("")
            self.reset_game()
            return self.reward, self.state, self.is_terminal




        self.step_number += 1
        self.episode_step_number += 1

        if self.safety_control.emergency_brake:
            if self.is_simulator:
                self.safety_control.disable_safety()
                time.sleep(0.3)
                self.control.backward_until_obstacle()
                self.safety_control.enable_safety()
                self.safety_control.unlock_brake()
                time.sleep(0.3)
            else:
                self.safety_control.disable_safety()
                time.sleep(0.6)
                self.control.backward_until_obstacle()
                time.sleep(0.4)
                self.safety_control.enable_safety()
                self.safety_control.unlock_brake()
                # if you select right/left from stop state, the real car turn the servo without moving..
                self.control.forward()
                time.sleep(0.4)

            if self.is_simulator:
                self.stuck_count = self.stuck_count + 1
                if self.stuck_count > 2:
                    print("Car stuck, resetting simulator position...")
                    self.control.reset_simulator()
                    self.stuck_count = 0

            position_index = float(self.position.position_index(self.position.car_position()))
            target_index = float(self.target.current_target_index())
            self.state = [[position_index], [target_index], self._get_car_state()]
            self.reward = -1
            self.is_terminal = True
            self.game_score += self.reward
            return self.reward, self.state, self.is_terminal

        self.stuck_count = 0
        reward = 0
        if action == 0:
            self.control.forward()
            reward = 0.2
        elif action == 1:
            self.control.right()
            reward = 0.05
        elif action == 2:
            self.control.left()
            reward = 0.05
        elif action == 3:
            self.control.lightly_right()
            reward = 0.1
        elif action == 4:
            self.control.lightly_left()
            reward = 0.1
        elif action == 5:
            self.control.stop()
            reward = -0.01
            self.car_stop_count += 1
        else:
            raise ValueError('`action` should be between 0 and ' + str(len(self.action_set)-1))

        if action != 5:
            self.car_stop_count = 0

        if self.car_stop_count > MAX_STOP:
            self.control.forward()

        if USE_VELOCITY_AS_REWARD:
            reward = self.sensors.get_car_linear_acelleration() * VELOCITY_NORMALIZATION * REWARD_SCALING

        if ADD_LIDAR_DISTANCE_REWARD:
            reward += min(list(self.sensors.get_lidar_ranges())) * LIDAR_DISTANCE_WEIGHT

        position_index = float(self.position.position_index(self.position.car_position()))
        target_index = float(self.target.current_target_index())
        self.state = [[position_index], [target_index], self._get_car_state()]
        self.reward = reward
        self.is_terminal = False
        self.game_score += self.reward

        return self.reward, self.state, self.is_terminal

    def reset_game(self):
        self.control.stop()

        position_index = float(self.position.position_index(self.position.car_position()))
        target_index = float(self.target.current_target_index())
        self.state = [[position_index], [target_index], self._get_car_state()]
        self.reward = 0
        if self.is_terminal:
            self.game_number += 1
            self.is_terminal = False

        self.game_score = 0
        self.episode_step_number = 0
        self.car_stop_count = 0

    def _get_car_state(self):
        current_data = list(self.sensors.get_lidar_ranges())
        return current_data


    def get_state_size(self):
        return len(self.state[2])

    def get_num_actions(self):
        return len(self.action_set)
    
    def get_game_number(self):
        return self.game_number
    
    def get_episode_step_number(self):
        return self.episode_step_number
    
    def get_step_number(self):
        return self.step_number
    
    def get_game_score(self):
        return self.game_score
