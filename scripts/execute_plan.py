#!/usr/bin/env python
# encoding: utf-8

import environment_api as api
import os

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))

if __name__ == "__main__":
    with open(os.path.join(root_path, 'problem.pddl.soln')) as file:
        current_state = api.get_current_state()
        for line in file:
            action = line[line.index(':')+1:].lower().split()
            if (action[0] == 'move'):
                robot = action[3]
                if 'house' in action[2]:
                    target = action[2].split('_')[1]
                    target = current_state['houses'][target]['deliver_loc']
                elif 'box' in action[2]:
                    target = action[2][:-5]
                    target = current_state['packages'][target]['load_loc'][0]
                else:
                    assert False
                
                actions = api.get_path(robot, target)

                for act in actions:
                    success, current_state = api.execute_action(act, [])
                    if success != 1:
                        print(act + ' failed!')
                        break

            elif (action[0] == 'load'):
                robot = action[4]
                success, current_state = api.execute_action(robot+'-load', [])
                if success != 1:
                    print('load failed!')
                    break

            elif (action[0] == 'deliver'):
                robot = action[5]
                success, current_state = api.execute_action(robot+'-deliver', [])
                if success != 1:
                    print('deliver failed!')
                    break

            else:
                print(action[0] + ' invalid action!')
                assert False

