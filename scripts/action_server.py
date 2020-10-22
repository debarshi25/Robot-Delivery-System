#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState,ModelStates
from std_msgs.msg import String
from group_16.srv import *
import numpy as np
import tf
import math
import copy
import json
import os
from collections import namedtuple
from heapq import heappush, heappop
import sets

class RobotActionsServer:

    def __init__(self, object_dict, robots, root_path, headless, random_seed=10):
        self.object_dict = object_dict
        self.failure = -1
        self.success = 1
        self.headless = headless
        self.status = String(data='Idle')
        self.model_state_publisher = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size = 10)
        self.action_publisher = {}
        self.status_publisher = {}
        for i in range(0, robots):
            robot = 'robot' + str(i)
            self.action_publisher[robot] = rospy.Publisher(robot+"/actions", String, queue_size=10)
            self.status_publisher[robot] = rospy.Publisher(robot+"/status", String, queue_size=10)
        self.random_seed = random_seed
        self.current_state = self.generate_init_state()
        self.action_config = self.load_action_config(root_path + '/action_config.json')
        self.direction_list = ["NORTH","EAST","SOUTH","WEST"]
        np.random.seed(self.random_seed)
        rospy.Service("execute_action", ActionMsg,self.execute_action)
        rospy.Service('get_all_actions', GetActions, self.get_all_actions)
        rospy.Service('get_possible_actions', GetPossibleActions, self.get_possible_actions)
        rospy.Service('get_possible_states', GetPossibleStates, self.get_possible_states)
        rospy.Service('get_reward', GetReward, self.get_reward)
        rospy.Service('is_terminal_state', IsTerminalState, self.is_terminal_state_handler)
        rospy.Service('get_current_state', GetInitialState, self.get_current_state)
        rospy.Service('get_path', GetPossibleActions, self.get_path)
        print("Action Server Initiated")
        with open(os.path.join(root_path, 'deliveries.json'), 'w') as file:
            json.dump(self.current_state, file)


    def generate_init_state(self):
        state = {}
        state['blocked_nodes'] = []
        state['robots'] = {}
        for robot in self.object_dict["robots"]:
            state['robots'][robot[0]] = {
                'loc': (robot[1], robot[2]),
                'orientation': 'EAST',
                'load_weight': 0.0,
                'capacity': 1.0
            }
            state['blocked_nodes'].append(state['robots'][robot[0]]['loc'])
        state['packages'] = {}
        house_ids = [i for i in range(0, len(self.object_dict["houses"]))]
        for package in self.object_dict["packages"]:
            package_id = 'cardboard_box_' + str(package.id)
            deliver_to = np.random.choice(house_ids)
            house_ids.remove(deliver_to)
            state['packages'][package_id] = {
                'loc': (package.pos_x, package.pos_y),
                'load_loc': [
                    (package.pos_x, package.pos_y+1)
                ],
                'size': package.size,
                'delivered': False,
                'deliver_to': str(deliver_to),
                'carried_by': None
            }
            state['blocked_nodes'].append(state['packages'][package_id]['loc'])
        state['houses'] = {}
        for house in self.object_dict["houses"]:
            state['houses'][house.id] = {
                'loc': (house.pos_x, house.pos_y),
                'region': ((house.pos_x - 1.0, house.pos_y + 2.0), (house.pos_x + 1.0, house.pos_y - 3.0)),
                'deliver_loc': (house.pos_x + 2.0, house.pos_y - 1.0)
            }
            for x in range(int(state['houses'][house.id]['region'][0][0]), int(state['houses'][house.id]['region'][1][0]+1)):
                for y in range(int(state['houses'][house.id]['region'][1][1]), int(state['houses'][house.id]['region'][0][1]+1)):
                    state['blocked_nodes'].append((x*1.0, y*1.0))
        return state


    def get_current_state(self, req):
        """
        This function will return current state of turtlebot3.
        """
        return json.dumps(self.current_state)


    def load_action_config(self, action_config_file):
        f = open(action_config_file)
        action_config = json.load(f)
        f.close()
        return action_config


    def get_turtlebot_location(self,robot,state):
        return state['robots'][robot]['loc'][0], state['robots'][robot]['loc'][1], state['robots'][robot]['orientation']


    def change_gazebo_state(self, package, target_transform):
        model_state_msg = ModelState()
        model_state_msg.model_name = package
        model_state_msg.pose.position.x = target_transform[0]
        model_state_msg.pose.position.y = target_transform[1]
        model_state_msg.pose.position.z = target_transform[2]
        self.model_state_publisher.publish(model_state_msg)

    '''
    def remove_edge(self, book_name):
        rospy.wait_for_service('remove_blocked_edge')
        try:
            remove_edge = rospy.ServiceProxy('remove_blocked_edge',RemoveBlockedEdgeMsg)
            _ = remove_edge(book_name)
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e


    def check_edge(self, x1, y1, x2, y2):
        rospy.wait_for_service('check_is_edge')
        try:
            check_is_edge = rospy.ServiceProxy('check_is_edge',CheckEdge)
            if x1 <= x2 and y1 <= y2:
                result = check_is_edge(x1,y1,x2,y2)
            else:
                result = check_is_edge(x2,y2,x1,y1)
            return result.value == 1
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e
    '''

    def is_terminal_state_handler(self, req):
        state = json.loads(req.state)
        return self.is_terminal_state(state)


    def is_terminal_state(self, state):
        # Terminal state is reached when all packages have been delivered
        is_terminal = True
        for package in state['packages'].keys():
            if not state['packages'][package]['delivered']:
                is_terminal = False
                break
        return is_terminal


    def get_all_actions(self, req):
        actions = []
        for robot in self.current_state['robots'].keys():
            actions += [robot+'-'+action for action in self.action_config.keys()]
        return ','.join(actions)


    def get_possible_actions(self, req):
        state = req.state
        actions = []

        for robot in self.current_state['robots'].keys():
            actions += [robot+'-'+action for action in self.action_config.keys()]

        pruned_actions = copy.deepcopy(actions)

        for action_str in actions:
            [robot, act] = action_str.split('-')
            success = 1

            if act == 'deliver':
                success, _ = self.execute_deliver(robot, state)
            elif act == 'load':
                success, _ = self.execute_load(robot, state)
            elif act == 'moveF':
                success, _ = self.execute_moveF(robot, state)
            
            if success == -1:
                pruned_actions.remove(action_str)

        return ','.join(pruned_actions)


    def get_possible_states(self, req):
        state = json.loads(req.state)
        [robot, action] = req.action.split('-')
        action_params = json.loads(req.action_params)

        next_states = {}
        i = 1
        for possible_action in self.action_config[action]['possibilities']:
            
            state_key = 'state_{}'.format(i)
            i += 1

            if possible_action == "noaction":
                next_states[state_key] = (state, self.action_config[action]['possibilities'][possible_action])
                continue
            
            # generate calling function
            calling_params = ["'" + robot + "'"]
            for param in self.action_config[possible_action]['params']:
                calling_params.append("'" + action_params[param] + "'")
            calling_params.append("'" + json.dumps(state) + "'")
            calling_function = "self.{}({})".format(self.action_config[possible_action]['function'], ','.join(calling_params))
            _, next_state = eval(calling_function)

            next_states[state_key] = (next_state, self.action_config[action]['possibilities'][possible_action])

        return json.dumps(next_states)


    def get_reward(self, req):
        state = json.loads(req.state)
        [_, action] = req.action.split('-')
        next_state = json.loads(req.next_state)

        if state == next_state:
            return self.action_config[action]['fail_reward']
        else:
            return self.action_config[action]['success_reward']
    

    def distance(self, loc1, loc2):
        return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])


    def get_path(self, req):
        request = json.loads(req.state)
        robot = request['robot']
        target = request['target']

        if (target[0], target[1]) in self.current_state['blocked_nodes']:
            print('no path exists!')
            return robot+'-noaction'

        action_list = []

        calling_params = ["'" + robot + "'"]
        calling_params.append("'" + json.dumps(self.current_state) + "'")

        actions = ['moveF', 'TurnCW', 'TurnCCW']
        fringe = []
        tie_breaker = 0
        init_state = copy.deepcopy(self.current_state)
        visited = sets.Set()
        visited.add(hash(str(init_state)))

        for new_action in actions:
            new_cost, new_state = eval("self.{}({})".format(self.action_config[new_action]['function'], ','.join(calling_params)))
            if new_cost < 0:
                continue
            new_cost += self.distance(new_state['robots'][robot]['loc'], target)
            heappush(fringe, (new_cost, tie_breaker, init_state, new_action, new_state))
            tie_breaker = tie_breaker + 1
        
        while len(fringe) > 0:
            cost, _, parent, action, state = heappop(fringe)

            if hash(str(state)) in visited:
                continue
            visited.add(hash(str(state)))
            
            if self.distance(state['robots'][robot]['loc'], target) == 0:
                action_list.append(robot+'-'+action)
                while parent != init_state:
                    parent, action = parent
                    action_list.append(robot+'-'+action)
                
                action_list.reverse()
                break
            
            new_parent = (parent, action)
            calling_params[1] = ("'" + json.dumps(state) + "'")

            for new_action in actions:
                new_cost, new_state = eval("self.{}({})".format(self.action_config[new_action]['function'], ','.join(calling_params)))
                if new_cost < 0:
                    continue
                new_cost += cost + self.distance(new_state['robots'][robot]['loc'], target)
                heappush(fringe, (new_cost, tie_breaker, new_parent, new_action, new_state))
                tie_breaker = tie_breaker + 1

        return ','.join(action_list)
    

    def execute_action(self, req):
        [robot, action] = req.action_name.split('-')
        params = json.loads(req.action_params)

        # No operations in terminal state
        if self.is_terminal_state(self.current_state):
            return -1, json.dumps(self.current_state)

        # Choose an action based on probabilities in action config
        chosen_action = np.random.choice(self.action_config[action]['possibilities'].keys(), 
                                         p=self.action_config[action]['possibilities'].values())

        if chosen_action == "noaction":
            return self.failure, json.dumps(self.current_state)

        # generate calling function
        calling_params = ["'" + robot + "'"]
        for param in self.action_config[chosen_action]['params']:
            calling_params.append("'" + params[param] + "'")
        calling_params.append("'" + json.dumps(self.current_state) + "'")
        calling_params.append('True')
        calling_function = "self.{}({})".format(self.action_config[chosen_action]['function'], ','.join(calling_params))
        success, next_state = eval(calling_function)
        
        # Update state
        self.current_state = copy.deepcopy(next_state)
        return success, json.dumps(next_state)


    def execute_deliver(self, robot, current_state, simulation=False):
        current_state = json.loads(current_state)
        robot_state = self.get_turtlebot_location(robot, current_state)
        next_state = copy.deepcopy(current_state)
        
        for package in current_state['packages'].keys():
            # Validate package carried by
            if current_state['packages'][package]['carried_by'] == robot:
                house = current_state['packages'][package]['deliver_to']
                # Validate robot is at delivery location
                if [robot_state[0],robot_state[1]] == current_state['houses'][house]['deliver_loc']:
                    carried = package
                    delivery = house

                    current_weight = current_state['robots'][robot]['load_weight']
                    next_weight = current_weight - current_state['packages'][package]['size']
                
                    goal_loc = current_state['houses'][delivery]['loc']
                    goal_loc[0] = goal_loc[0] + 1.0
                    goal_loc[1] = goal_loc[1] - 1.0
                    # Update gazebo environment if needed
                    if simulation and not self.headless:
                        self.change_gazebo_state(carried, goal_loc + [0])
                        rospy.Rate(1).sleep()
                        
                    self.status_publisher[robot].publish(self.status)

                    # Update state
                    next_state['packages'][carried]['loc'] = (goal_loc[0], goal_loc[1])
                    next_state['packages'][carried]['delivered'] = True
                    next_state['packages'][carried]['carried_by'] = None
                    next_state['robots'][robot]['load_weight'] = next_weight
                            
                    return self.success, next_state
        
        self.status_publisher[robot].publish(self.status)
        return self.failure, next_state


    def execute_load(self, robot, current_state, simulation=False):
        current_state = json.loads(current_state)
        robot_state = self.get_turtlebot_location(robot, current_state)
        next_state = copy.deepcopy(current_state)

        for package in current_state['packages'].keys():
            if [robot_state[0],robot_state[1]] in current_state['packages'][package]['load_loc']:
                if current_state['packages'][package]['carried_by'] is None:
                    current_weight = current_state['robots'][robot]['load_weight']
                    next_weight = current_weight + current_state['packages'][package]['size']
                    if next_weight <= 1.0:
                        if simulation and not self.headless:
                            self.change_gazebo_state(package, list(current_state['robots'][robot]['loc'][:2])+[2])
                            rospy.Rate(1).sleep()

                        # Update state
                        next_state['packages'][package]['carried_by'] = robot
                        next_state['blocked_nodes'].remove(current_state['packages'][package]['loc'])
                        next_state['packages'][package]['loc'] = (robot_state[0], robot_state[1])
                        next_state['robots'][robot]['load_weight'] = next_weight

                        return self.success, next_state

        self.status_publisher[robot].publish(self.status)
        return self.failure, next_state


    def execute_moveF(self, robot, current_state, simulation=False):
        current_state = json.loads(current_state)
        robot_state = self.get_turtlebot_location(robot, current_state)
        next_state = copy.deepcopy(current_state)
        x1 = robot_state[0]
        y1 = robot_state[1]

        # Get new location
        if "EAST" in robot_state[2]:
            x2 = x1 + 1.0
            y2 = y1
        elif "WEST" in robot_state[2]:
            x2 = x1 - 1.0
            y2 = y1
        elif "NORTH" in robot_state[2]:
            x2 = x1
            y2 = y1 + 1.0
        else:
            x2 = x1
            y2 = y1 - 1.0

        # Check if that edge isn't blocked
        if [x2, y2] not in current_state['blocked_nodes']:
            action_str = "MoveF"

            # Make bot move if simulating in gazebo
            if simulation and not self.headless:
                self.action_publisher[robot].publish(String(data=action_str))
                rospy.wait_for_message(robot+"/status",String)
            
            # Update State
            next_state['blocked_nodes'].remove([x1, y1])
            next_state['robots'][robot]['loc'] = (x2, y2)
            next_state['blocked_nodes'].append((x2, y2))

            return self.success, next_state
        else:
            return self.failure, next_state


    def execute_TurnCW(self, robot, current_state, simulation=False):
        current_state = json.loads(current_state)
        next_state = copy.deepcopy(current_state)

        # Make bot move if simulating in gazebo
        if simulation and not self.headless:
            action_str = "TurnCW"
            self.action_publisher[robot].publish(String(data=action_str))
            rospy.wait_for_message(robot+"/status",String)

        # Update state
        current_orientation = current_state['robots'][robot]['orientation']
        new_orientation = self.direction_list[(self.direction_list.index(current_orientation) + 1)%4]
        next_state['robots'][robot]['orientation'] = new_orientation

        return self.success, next_state


    def execute_TurnCCW(self, robot, current_state, simulation=False):
        current_state = json.loads(current_state)
        next_state = copy.deepcopy(current_state)
        
        # Make bot move if simulating in gazebo
        if simulation and not self.headless:
            action_str = "TurnCCW"
            self.action_publisher[robot].publish(String(data=action_str))
            rospy.wait_for_message(robot+"/status",String)
        
        # Update state
        current_orientation = current_state['robots'][robot]['orientation']
        new_orientation = self.direction_list[(self.direction_list.index(current_orientation) - 1)%4]
        next_state['robots'][robot]['orientation'] = new_orientation

        return self.success, next_state


if __name__ == "__main__":
    object_dict = None
    RobotActionsServer(object_dict)
