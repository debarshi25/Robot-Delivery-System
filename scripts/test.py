#!/usr/bin/env python
# encoding: utf-8

import argparse
import environment_api as api

parser = argparse.ArgumentParser()
parser.add_argument('-command', type=str)
parser.add_argument('-action', type=str)
parser.add_argument('-robot', type=str)
parser.add_argument('-package', default=None, type=str)
parser.add_argument('-house', default=None, type=str)

if __name__ == "__main__":
    args = parser.parse_args()

    if args.command == 'get_current_state':
        print(api.get_current_state())
    elif args.command == 'is_terminal_state':
        current_state = api.get_current_state()
        print(api.is_terminal_state(current_state))
    elif args.command == 'reset_world':
        print(api.reset_world())
    elif args.command == 'get_all_actions':
        print(api.get_all_actions())
    elif args.command == 'get_possible_actions':
        current_state = api.get_current_state()
        print(api.get_possible_actions(current_state))
    elif args.command == 'get_possible_states':
        current_state = api.get_current_state()
        for robot in current_state['robots'].keys():
            for action in api.get_possible_actions(current_state, robot):
                print(api.get_possible_states(current_state, action, {'robot':robot}))
    elif args.command == 'execute_action':
        current_state = api.get_current_state()
        success, next_state = api.execute_action(args.action, {})
        print(success)
        print(api.get_reward(current_state, args.action, next_state))
    elif args.command == 'get_path':
        print(api.get_path('robot0', (0.0,-2.0)))

