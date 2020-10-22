#!/usr/bin/env python

from group_16.srv import *
import rospy
import sys
import argparse
import time
from action_server import RobotActionsServer 
import pickle
import copy
import os

from neighborhood import Neighborhood
import problem_generator
import json

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
books = None
neighborhoodInfo = None
neighborhoodInfoCopy = None
parser = argparse.ArgumentParser()
parser.add_argument('-houses', help='number of houses per street', metavar='3', default=3, type=int)
parser.add_argument('-packages', help='number of packages', metavar='8', default=8, type=int)
parser.add_argument('-robots', help='number of robots', metavar='1', default=1, type=int)
parser.add_argument('-streets', help='number of streets', metavar='4', default=4, type=int)
parser.add_argument('-headless', help='whether to run without gazebo', default=False, type=bool)
parser.add_argument('-seed', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)
robot_action_server = None


def check_is_edge(req):
	"""
	This function checks if two points are connected via edge or not.
	"""
	global neighborhoodInfo
	edge = (req.x1,req.y1,req.x2,req.y2)
	for edge_point in edge:
		if edge_point < neighborhoodInfo.grid_start or edge_point > neighborhoodInfo.grid_dimension * 0.5:
			return 0
	if edge in neighborhoodInfo.blocked_edges or (edge[2],edge[3],edge[0],edge[1]) in neighborhoodInfo.blocked_edges:
		return 0
	else:
		return 1


def handle_reset_world(req):
	global neighborhoodInfo
	neighborhoodInfo = copy.deepcopy(neighborhoodInfoCopy)
	robot_action_server.current_state = robot_action_server.generate_init_state()
	return 1


def remove_blocked_edge(req):
	bookname = req.bookname
	global books
	global neighborhoodInfo
	location_of_blocked_edge_list = books["books"][bookname]["load_loc"]
	if location_of_blocked_edge_list[0][0] <= location_of_blocked_edge_list[1][0] and location_of_blocked_edge_list[0][1] <= location_of_blocked_edge_list[1][1]:
		blocked_edge = (location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1], location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1])
	else:
		blocked_edge = (location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1], location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1])
	neighborhoodInfo.blocked_edges.remove(blocked_edge)
	return "1"


def server():
	rospy.Service('remove_blocked_edge', RemoveBlockedEdgeMsg, remove_blocked_edge)
	rospy.Service('check_is_edge', CheckEdge, check_is_edge)
	rospy.Service('reset_world', ResetWorldMsg, handle_reset_world)
	print("Ready!")
	rospy.spin()


if __name__ == "__main__":
	args = parser.parse_args()
	print('houses: ' + str(args.houses*args.streets))
	print('packages: ' + str(args.packages))
	print('streets: ' + str(args.streets))
	print('robots: ' + str(args.robots))
	neighborhoodInfo = Neighborhood(12, 0.5)
	objects = neighborhoodInfo.generate_blocked_edges(args.houses, args.streets, args.packages, args.seed, root_path)
	objects = neighborhoodInfo.generate_launch(args.robots, objects, root_path)
	neighborhoodInfoCopy = copy.deepcopy(neighborhoodInfo)
	rospy.init_node('server')
	robot_action_server = RobotActionsServer(objects, args.robots, root_path, args.headless, args.seed)
	path = root_path + "/problem.pddl"
	with open(root_path + "/deliveries.json") as json_file:
		deliveries = json.load(json_file)
	problem_generator.write_pddl(path, deliveries)
	server()
