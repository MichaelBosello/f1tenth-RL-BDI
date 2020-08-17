#!/usr/bin/env python3
#
from threading import Thread
import sys
import numpy as np
import os
import random
import time
import argparse
import datetime
import tensorflow as tf

from car_driver.dqn import DeepQNetwork
from car_driver.replay import ReplayMemory, Sample
from car_driver.state import State

class DqnAgent():
    def __init__(self, num_action, state_size, params={}, agent_id = None):
        self.agent_id = agent_id
        self.__parse_argument()
        self.__setup(num_action, state_size)

    #################################
    # parameters
    #################################
    def __parse_argument(self):
        parser = argparse.ArgumentParser()
        # agent parameters
        parser.add_argument("--learning-rate", type=float, default=0.00042, help="learning rate of the NN")
        parser.add_argument("--gamma", type=float, default=0.98, help="""gamma [0, 1] is the discount factor. It determines the importance of future rewards.
                                        A factor of 0 will make the agent consider only immediate reward, a factor approaching 1 will make it strive for a long-term high reward""")
        parser.add_argument("--epsilon", type=float, default=1, help="]0, 1]for epsilon greedy train")
        parser.add_argument("--epsilon-decay", type=float, default=0.9999342, help="]0, 1] every step epsilon = epsilon * decay, in order to decrease constantly")
        parser.add_argument("--epsilon-min", type=float, default=0.1, help="epsilon with decay doesn't fall below epsilon min")
        parser.add_argument("--eval-epsilon", type=float, default=0.01, help="epsilon used in evaluation")
        parser.add_argument("--batch-size", type=float, default=32, help="size of the batch used in gradient descent")

        parser.add_argument("--observation-steps", type=int, default=400, help="train only after this many steps (1 step = [history-length] frames)")
        parser.add_argument("--target-model-update-freq", type=int, default=500, help="how often (in steps) to update the target model")
        parser.add_argument("--model", help="tensorflow model directory to initialize from (e.g. run/model)")
        parser.add_argument("--history-length", type=int, default=2, help="(>=1) length of history used in the dqn. An action is performed [history-length] time")
        parser.add_argument("--repeat-action", type=int, default=2, help="(>=0) actions are repeated [repeat-action] times. Unlike history-length, it doesn't increase the network size")
        parser.add_argument("--gpu-time", type=int, default=0.0042, help="""waiting time (seconds) between actions when agent is not training (observation steps/evaluation).
                                        It should be the amount of time used by your CPU/GPU to perform a training sweep. It is needed to have the same states and rewards as
                                        training takes time and the environment evolves indipendently""")
        parser.add_argument("--slowdown-cycle", type=bool, default=True, help="add a sleep equal to [gpu-time] in the training cycle")
        parser.add_argument("--show-cycle-time", type=bool, default=False, help="it prints the seconds used in one step, useful to update the above param")
        # lidar pre-processing
        parser.add_argument("--reduce-lidar-data", type=int, default=30, help="lidar data are grouped by taking the min of [reduce-lidar-data] elements")
        parser.add_argument("--cut-lidar-data", type=int, default=8, help="N element at begin and end of lidar data are cutted. Executed after the grouping")
        parser.add_argument("--max-distance-norm", type=float, default=20, help="divide lidar elems by [max-distance-norm] to normalize between [0, 1]")
        parser.add_argument("--lidar-reduction-method", choices=['avg', 'max', 'min', 'sampling'], default='avg', type=str.lower, help="method used to aggregate lidar data")
        parser.add_argument("--lidar-float-cut", type=int, default=-1, help="how many decimals of lidar ranges to take. -1 for no cutting")

        parser.add_argument("--lidar-to-image", type=bool, default=False, help="if true, an image of borders is built from lidar ranges and it is used as state")
        parser.add_argument("--show-image", type=bool, default=False, help="show the agent view. [lidar-to-image] must be true to have effect")
        parser.add_argument("--image-width", type=int, default=84, help="the width of the image built from lidar data. Applicable if [lidar-to-image] is true")
        parser.add_argument("--image-height", type=int, default=84, help="the height of the image built from lidar data. Applicable if [lidar-to-image] is true")
        parser.add_argument("--image-zoom", type=int, default=2, help="""zoom lidar image to increase border separation.
                                        It must be appropriate for the track max distance and image size otherwise out-of-bound exception will be casted""")
        # train parameters
        parser.add_argument("--train-epoch-steps", type=int, default=3500, help="how many steps (1 step = [history-length] frames) to run during a training epoch")
        parser.add_argument("--eval-epoch-steps", type=int, default=500, help="how many steps (1 step = [history-length] frames) to run during an eval epoch")
        parser.add_argument("--replay-capacity", type=int, default=100000, help="how many states to store for future training")
        parser.add_argument("--prioritized-replay", action='store_true', help="prioritize interesting states when training (e.g. terminal or non zero rewards)")
        parser.add_argument("--compress-replay", action='store_true', help="if set replay memory will be compressed with blosc, allowing much larger replay capacity")
        parser.add_argument("--save-model-freq", type=int, default=2000, help="save the model every X steps")
        parser.add_argument("--logging", type=bool, default=True, help="enable tensorboard logging")
        
        self.args = parser.parse_args()

        if self.agent_id == "turn_left" or self.agent_id == "go_forward":
            self.args.learning_rate = 0.00044
            self.args.epsilon_decay = 0.99993
            self.args.target_model_update_freq = 400
            self.args.repeat_action = 3
        if self.agent_id == "follow_street":
            pass

        print('Arguments: ', (self.args))

    #################################
    # setup
    #################################
    def __setup(self, num_action, state_size):
        State.setup(self.args)
        self.num_action = num_action
        self.state_size = State.size_after_processing(state_size)
        base_output_dir = 'run-out-' + time.strftime("%Y-%m-%d-%H-%M-%S")
        os.makedirs(base_output_dir)

        tensorboard_dir = base_output_dir + "/tensorboard/"
        os.makedirs(tensorboard_dir)
        self.summary_writer = tf.summary.create_file_writer(tensorboard_dir)
        with self.summary_writer.as_default():
            tf.summary.text('params', str(self.args), step=0)

        self.replay_memory = ReplayMemory(base_output_dir, self.args)
        self.dqn = DeepQNetwork(self.num_action, self.state_size,
                                self.replay_memory, base_output_dir, tensorboard_dir, self.args)

        self.train_epsilon = self.args.epsilon
        self.start_time = datetime.datetime.now()
        self.train_episodes = 0
        self.eval_episodes = 0
        self.episode_train_reward_list = []
        self.episode_eval_reward_list = []
        self.episode_losses = []
        self.steps = 0
        self.step_start = 0
        self.repeat_step_count = 0
        self.episode_score = 0
        self.save_net = False
        self.old_state = None
        self.action = None
        self.is_epoch_training = self.args.train_epoch_steps > 0
        self.start_time_cycle = None
        self.time_list_train = []
        self.time_list_eval = []

    #################################
    # logging
    #################################
    def __log(self, is_training):
        episode_time = datetime.datetime.now() - self.start_time

        if is_training:
            self.train_episodes += 1
            self.episode_train_reward_list.insert(0, self.episode_score)
            if len(self.episode_train_reward_list) > 100:
                self.episode_train_reward_list = self.episode_train_reward_list[:-1]
            avg_rewards = np.mean(self.episode_train_reward_list)

            episode_avg_loss = 0
            if self.episode_losses:
                episode_avg_loss = np.mean(self.episode_losses)
                self.episode_losses = []

            log = ('Episode %d ended with score: %.2f (%s elapsed) (step: %d). Avg score: %.2f Avg loss: %.5f' %
                (self.train_episodes, self.episode_score, str(episode_time),
                self.steps, avg_rewards, episode_avg_loss))
            print(log)
            print("   epsilon " + str(self.train_epsilon))
            if self.args.logging:
                with self.summary_writer.as_default():
                    tf.summary.scalar('train episode reward', self.episode_score, step=self.train_episodes)
                    tf.summary.scalar('train avg reward(100)', avg_rewards, step=self.train_episodes)
                    tf.summary.scalar('average loss', episode_avg_loss, step=self.train_episodes)
                    tf.summary.scalar('epsilon', self.train_epsilon, step=self.train_episodes)
                    tf.summary.scalar('steps', self.steps, step=self.train_episodes)
        else:
            self.eval_episodes += 1
            self.episode_eval_reward_list.insert(0, self.episode_score)
            if len(self.episode_eval_reward_list) > 100:
                self.episode_eval_reward_list = self.episode_eval_reward_list[:-1]
            avg_rewards = np.mean(self.episode_eval_reward_list)

            log = ('Eval %d ended with score: %.2f (%s elapsed) (step: %d). Avg score: %.2f' %
                (self.eval_episodes, self.episode_score, str(episode_time),
                self.steps, avg_rewards))
            print(log)
            if self.args.logging:
                with self.summary_writer.as_default():
                    tf.summary.scalar('eval episode reward', self.episode_score, step=self.eval_episodes)
                    tf.summary.scalar('eval avg reward(100)', avg_rewards, step=self.eval_episodes)

    def __choose_action(self, state, is_training):
        # epsilon selection and update
        if is_training:
            epsilon = self.train_epsilon
            if self.train_epsilon > self.args.epsilon_min:
                self.train_epsilon = self.train_epsilon * self.args.epsilon_decay
                if self.train_epsilon < self.args.epsilon_min:
                    self.train_epsilon = self.args.epsilon_min
        else:
            epsilon = self.args.eval_epsilon

        # action selection
        if state is None or random.random() < epsilon:
            action = random.randrange(self.num_action)
        else:
            action = self.dqn.inference(state.get_data())[0]
        return action

    def epoch_step(self, state, reward, is_terminal):
        if self.args.show_cycle_time:
            if not is_terminal:
                if self.start_time_cycle is not None:
                    cycle_time = (datetime.datetime.now() - self.start_time_cycle).total_seconds()
                    if self.is_epoch_training:
                        self.time_list_train.insert(0, cycle_time)
                        if len(self.time_list_train) > 100:
                            self.time_list_train = self.time_list_train[:-1]
                        print("Cycle time (train): %fs, Avg time:%fs" % (cycle_time, np.mean(self.time_list_train)))
                    else:
                        self.time_list_eval.insert(0, cycle_time)
                        if len(self.time_list_eval) > 100:
                            self.time_list_eval = self.time_list_eval[:-1]
                        print("Cycle time (eval): %fs, Avg time:%fs" % (cycle_time, np.mean(self.time_list_eval)))
                self.start_time_cycle = datetime.datetime.now()
            else:
                self.start_time_cycle = None

        self.steps += 1
        self.repeat_step_count += 1
        self.episode_score += reward
        if self.steps % self.args.save_model_freq == 0:
            self.save_net = True

        if self.is_epoch_training:
            action = self.train(state, reward, is_terminal)
        else:
            action = self.inference(state, reward, is_terminal)

        if is_terminal:
            self.repeat_step_count = 0
            self.episode_score = 0
            if self.is_epoch_training and self.args.eval_epoch_steps > 0:
                if self.steps - self.step_start >= self.args.train_epoch_steps:
                    self.is_epoch_training = False
                    self.step_start = self.steps
            if not self.is_epoch_training and self.args.train_epoch_steps > 0:
                if self.steps - self.step_start >= self.args.eval_epoch_steps:
                    self.is_epoch_training = True
                    self.step_start = self.steps

        return action

    def train(self, state, reward, is_terminal):
        if (self.repeat_step_count >= self.args.history_length * (self.args.repeat_action + 1)
                or self.action is None or is_terminal):
            self.repeat_step_count = 0
            # Record experience in replay memory
            if self.old_state is not None:
                current_state = self.old_state.state_by_adding_data(state)
                if self.action is not None:
                    self.replay_memory.add_sample(Sample(
                        self.old_state, self.action, reward, current_state, is_terminal))
                self.old_state = current_state
            else:
                self.old_state = State().state_by_adding_data(state)
            self.action = self.__choose_action(self.old_state, True)

        # train
        if self.steps > self.args.observation_steps:
            batch = self.replay_memory.draw_batch(self.args.batch_size)
            loss = self.dqn.train(batch, self.steps)
            self.episode_losses.append(loss)
            if self.args.slowdown_cycle:
                time.sleep(self.args.gpu_time)
        else:
            time.sleep(self.args.gpu_time)

        if is_terminal:
            self.old_state = None
            self.action = None
            if self.save_net:
                self.dqn.save_network()
                self.save_net = False
            self.__log(True)
            return 0

        return self.action


    def inference(self, state, reward, is_terminal):
        if (self.repeat_step_count >= self.args.history_length * (self.args.repeat_action + 1)
                or self.action is None):
            self.repeat_step_count = 0
            if self.old_state is not None:
                self.old_state = self.old_state.state_by_adding_data(state)
            else:
                self.old_state = State().state_by_adding_data(state)
            self.action = self.__choose_action(self.old_state, False)
        time.sleep(self.args.gpu_time)

        if is_terminal:
            self.old_state = None
            self.action = None
            self.__log(False)
            return 0

        return self.action