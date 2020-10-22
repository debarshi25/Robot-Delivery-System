#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Chirav Dave", "Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import pickle

def write_objects(fhandle, object_dict):
	house_list = object_dict["houses"].keys()
	house_loc_list = ["house_" + house_name + "_iloc" for house_name in house_list]
	package_list = object_dict["packages"].keys()
	package_loc_list = [package_name + "_iloc" for package_name in package_list]
	robot_list = object_dict["robots"].keys()
	robot_loc_list = [robot_name + "_init_iloc" for robot_name in robot_list]
	house_list_str = " ".join("house_" + house for house in house_list) + " - house"
	house_loc_str = " ".join(house_loc for house_loc in house_loc_list)  + " - location"
	package_list_str = " ".join(package_name for package_name in package_list) + " - package"
	package_loc_str = " ".join(package_loc for package_loc in package_loc_list) + " - location"
	robot_list_str = " ".join(robot_name for robot_name in robot_list) + " - robot"
	robot_loc_str = " ".join(robot_loc for robot_loc in robot_loc_list) + " - location"
	#subject_set = set()
	#for book in book_list:
	#	subject_set.add(object_dict["books"][book]["subject"].replace(" ","_"))
	#subject_str = " ".join(sub_name for sub_name in subject_set) + " - subject"
	fhandle.write("(:objects" + "\n")
	fhandle.write(robot_list_str + "\n")
	fhandle.write(robot_loc_str + "\n")
	fhandle.write(package_list_str + "\n")
	fhandle.write(house_list_str + "\n")
	fhandle.write(package_loc_str + "\n")
	fhandle.write(house_loc_str + "\n")
	#fhandle.write(subject_str + "\n")
	fhandle.write("n0 n25 n50 n75 n100 - weight" + "\n")
	fhandle.write("False True - binary" + "\n")
	fhandle.write(")" + "\n")
	return robot_list, robot_loc_list, package_list, package_loc_list, house_list, house_loc_list

def write_init_state(fhandle, object_dict, robot_list, robot_loc_list, package_list, package_loc_list, house_list, house_loc_list):
	fhandle.write("(:init"+"\n")
	for i in range(len(package_list)):
		fhandle.write("(Package_At {} {})".format(package_list[i], package_loc_list[i]) + "\n")
		fhandle.write("(= (x_loc {}) {})".format(package_loc_list[i], int(object_dict['packages'][package_list[i]]['loc'][0])) + '\n')
		fhandle.write("(= (y_loc {}) {})".format(package_loc_list[i], int(object_dict['packages'][package_list[i]]['loc'][1])) + '\n')
	for i in range(len(house_list)):
		fhandle.write("(House_At {} {})".format("house_" + house_list[i], house_loc_list[i]) + "\n")
		fhandle.write("(= (x_loc {}) {})".format(house_loc_list[i], int(object_dict['houses'][house_list[i]]['loc'][0])) + '\n')
		fhandle.write("(= (y_loc {}) {})".format(house_loc_list[i], int(object_dict['houses'][house_list[i]]['loc'][1])) + '\n')
	for package in package_list:
		#fhandle.write("(Book_Subject {} {})".format(book,object_dict["books"][book]["subject"].replace(" ","_")) + "\n")
		size = object_dict["packages"][package]["size"]
		if size == 0:
			size = "n0"
		elif size == 0.25:
			size = "n25"
		elif size == 0.5:
			size = "n50"
		elif size == 0.75:
			size = "n75"
		elif size == 1:
			size = "n100"
		fhandle.write("(Package_Size {} {})".format(package, size) + "\n")
		fhandle.write("(Package_To {} {})".format(package, "house_" + str(object_dict["packages"][package]["deliver_to"]) + "_iloc") + "\n")
		fhandle.write("(Delivery_Status {} {})".format(package, object_dict["packages"][package]["delivered"]) + "\n")
	#for house in house_list:
		#fhandle.write("(Bin_Subject {} {})".format(bin_name,object_dict["bins"][bin_name]["subject"].replace(" ","_")) + "\n")
		#fhandle.write("(House_Size {} {})".format("house_" + house, object_dict["houses"][house]["size"]) + "\n")
	for i in range(len(robot_list)):
		fhandle.write("(Robot_At {} {})".format(robot_list[i], robot_loc_list[i]) + "\n")
		fhandle.write("(= (x_loc {}) {})".format(robot_loc_list[i], int(object_dict['robots'][robot_list[i]]['loc'][0])) + '\n')
		fhandle.write("(= (y_loc {}) {})".format(robot_loc_list[i], int(object_dict['robots'][robot_list[i]]['loc'][1])) + '\n')
	for robot in robot_list:
		load_weight = object_dict["robots"][robot]["load_weight"]
		if load_weight == 0:
			load_weight = "n0"
		elif load_weight == 0.25:
			load_weight = "n25"
		elif load_weight == 0.5:
			load_weight = "n50"
		elif load_weight == 0.75:
			load_weight = "n75"
		elif load_weight == 1:
			load_weight = "n100"
		fhandle.write("(Robot_Load {} {})".format(robot, load_weight) + "\n")
		fhandle.write("(= (action_cost {}) 0)".format(robot))
	fhandle.write(")" + "\n")





def write_pddl(path, object_dict):
	fhandle = open(path,"w")
	fhandle.write("(define (problem robot_delivery_system)\n")
	fhandle.write("(:domain neighborhood)\n")
	robot_list, robot_loc_list, package_list, package_loc_list, house_list, house_loc_list = write_objects(fhandle, object_dict)
	write_init_state(fhandle, object_dict, robot_list, robot_loc_list, package_list, package_loc_list, house_list, house_loc_list)
	fhandle.write("(:goal (forall (?package - package) (Delivery_Status ?package True)))" + "\n")
	fhandle.write(")")
	fhandle.close()


if __name__ == "__main__":
	# # object_dict = {"books" : { "b1" : { "size" : "large","subject" : "s1" , "loc" : (1,2)},\
	# 							"b2" : { "size" : "small","subject" : "s2" , "loc" : (2,2)}\
	# 						},\
	# 				"bins" : {"bin1" : { "size" : "large","subject" : "s1" , "loc" : (3,2)},\
	# 							"bin2" : { "size" : "small","subject" : "s2" , "loc" : (4,2)}\
	# 						}\
	# 			}
	object_dict = pickle.load(open("/home/naman/catkin_ws/src/planning/object_dict.p","rb"))
	fhandle = open("temp_problem.pddl","w")
	write_pddl(fhandle,object_dict)
