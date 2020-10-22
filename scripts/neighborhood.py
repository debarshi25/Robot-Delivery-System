import argparse
import random
import copy

import numpy as np

class Package:
	# __init__
	#
	# @param	id		str
	# @param	pos_x	float
	# @param	pos_y	float
	# @param	size	float (0.0 < size <= 1.0)
	def __init__(self, id, pos_x, pos_y, size):
		self.id = id
		self.pos_x = pos_x
		self.pos_y = pos_y
		self.size = size


class House:
	# __init__
	#
	# @param	id		int
	# @param	pos_x	float
	# @param	pos_y	float
	def __init__(self, id, pos_x, pos_y):
		self.id = id
		self.pos_x = pos_x
		self.pos_y = pos_y


class Neighborhood:

	def __init__(self, grid_dimension, myscale=0.5):
		self.grid_dimension = grid_dimension
		self.grid_start = 0
		self.myscale = myscale
		self.blocked_edges = set()


	def __deepcopy__(self, memodict={}):
		new_neighborhood = Neighborhood(self.grid_dimension)
		new_neighborhood.blocked_edges = copy.deepcopy(self.blocked_edges)
		return new_neighborhood


	def copy_empty_world(self,root_path):
		f_in = open(root_path+'/worlds/empty_world.sdf', 'r')
		f_out = open(root_path+'/worlds/neighborhood.sdf', 'w')
		for line in f_in:
			f_out.write(line)
		f_in.close()
		return f_out


	def add_box(self, f_out, box):
		f_out.write('''
			<model name='cardboard_box_{0}'>
				<pose frame=''>{1} {2} 0.14951 2e-06 -2e-06 -0</pose>
				<scale>{3} {3} {3}</scale>
				<link name='link'>
					<pose frame=''>{1} {2} 0.14951 2e-06 -2e-06 -0</pose>
					<velocity>0 0 0 0 -0 0</velocity>
					<acceleration>0 1.1e-05 0 -7.2e-05 1e-06 0</acceleration>
					<wrench>0 2.2e-05 0 0 -0 0</wrench>
				</link>
			</model>\n'''.format(box.id, box.pos_x, box.pos_y, box.size/4.0))

	
	def add_box_description(self, f_out, box):
		f_out.write('''
			<model name='cardboard_box_{0}'>
				<pose frame=''>{1} {2} 0.15 0 -0 0</pose>
				<link name='link'>
					<inertial>
						<mass>0.01</mass>
						<inertia>
							<ixx>0.0416667</ixx>
							<ixy>0</ixy>
							<ixz>0</ixz>
							<iyy>0.0566667</iyy>
							<iyz>0</iyz>
							<izz>0.0683333</izz>
						</inertia>
					</inertial>
					<collision name='collision'>
						<geometry>
							<box>
								<size>0.5 0.4 0.3</size>
							</box>
						</geometry>
						<surface>
							<friction>
								<ode>
									<mu>1</mu>
									<mu2>1</mu2>
								</ode>
								<torsional>
									<ode/>
								</torsional>
							</friction>
							<contact>
								<ode>
									<kp>1e+07</kp>
									<kd>1</kd>
									<min_depth>0.001</min_depth>
									<max_vel>0.1</max_vel>
								</ode>
							</contact>
							<bounce/>
						</surface>
						<max_contacts>10</max_contacts>
					</collision>
					<visual name='visual'>
						<pose frame=''>0 0 -0.15 0 -0 0</pose>
						<geometry>
							<mesh>
								<uri>model://cardboard_box/meshes/cardboard_box.dae</uri>
								<scale>{3} {3} {3}</scale>
							</mesh>
						</geometry>
					</visual>
					<self_collide>0</self_collide>
					<kinematic>0</kinematic>
					<gravity>1</gravity>
				</link>
			</model>)\n'''.format(box.id, box.pos_x, box.pos_y, box.size/4.0))


	def add_house(self, f_out, house):
		f_out.write('''
			<model name='House_{0}'>
				<pose frame=''>{1} {2} 0 0 -0 0</pose>
				<scale>0.5 0.5 0.5</scale>
				<link name='link'>
					<pose frame=''>{1} {2} 0 0 0 1.57</pose>
					<velocity>0 0 0 0 -0 0</velocity>
					<acceleration>0 0 0 0 -0 0</acceleration>
					<wrench>0 0 0 0 -0 0</wrench>
				</link>
			</model>\n'''.format(house.id, house.pos_x, house.pos_y))


	def add_house_description(self, f_out, house):
		f_out.write('''
			<model name='House_{0}'>
				<static>1</static>
				<link name='link'>
					<collision name='collision'>
						<geometry>
							<mesh>
								<uri>model://house_3/meshes/house_3.dae</uri>
								<scale>0.5 0.5 0.5</scale>
							</mesh>
						</geometry>
						<max_contacts>10</max_contacts>
						<surface>
							<contact>
								<ode/>
							</contact>
							<bounce/>
							<friction>
								<torsional>
									<ode/>
								</torsional>
								<ode/>
							</friction>
						</surface>
					</collision>
					<visual name='visual'>
						<geometry>
							<mesh>
								<uri>model://house_3/meshes/house_3.dae</uri>
								<scale>0.5 0.5 0.5</scale>
							</mesh>
						</geometry>
						<material>
							<script>
								<uri>model://house_3/materials/scripts</uri>
								<uri>model://house_3/materials/textures</uri>
								<name>House_3/Diffuse</name>
							</script>
						</material>
					</visual>
					<self_collide>0</self_collide>
					<kinematic>0</kinematic>
					<gravity>1</gravity>
				</link>
				<pose frame=''>{1} {2} 0 0 -0 0</pose>
			</model>\n'''.format(house.id, house.pos_x, house.pos_y))


	def add_wall(self, f_out):
		f_out.write('''
			<model name='nist_maze_wall_120'>
				<pose frame=''>-3.2944 0.18992 0 0 -0 0</pose>
				<scale>1 1 1</scale>
				<link name='nist_maze_wall_120_link'>
					<pose frame=''>-3.2944 0.18992 0 0 -0 0</pose>
					<velocity>0 0 0 0 -0 0</velocity>
					<acceleration>0 0 0 0 -0 0</acceleration>
					<wrench>0 0 0 0 -0 0</wrench>
				</link>
			</model>\n''')

	
	def add_wall_description(self, f_out):
		f_out.write('''
			<model name='nist_maze_wall_120'>
				<static>1</static>
				<link name='nist_maze_wall_120_link'>
					<pose frame=''>0 0 0 0 -0 0</pose>
					<collision name='collision'>
						<geometry>
							<mesh>
								<uri>model://nist_maze_wall_120/meshes/nist_maze_wall_120.dae</uri>
								<scale>1 1 1</scale>
							</mesh>
						</geometry>
						<max_contacts>10</max_contacts>
						<surface>
							<contact>
								<ode/>
							</contact>
							<bounce/>
							<friction>
								<torsional>
									<ode/>
								</torsional>
								<ode/>
							</friction>
						</surface>
					</collision>
					<visual name='visual'>
						<geometry>
							<mesh>
								<uri>model://nist_maze_wall_120/meshes/nist_maze_wall_120.dae</uri>
								<scale>1 1 1</scale>
							</mesh>
						</geometry>
					</visual>
					<self_collide>0</self_collide>
					<kinematic>0</kinematic>
					<gravity>1</gravity>
				</link>
				<pose frame=''>-3.2944 0.18992 0 0 -0 0</pose>
			</model>\n''')


	def add_blocked_edges(self, x, y):
		blocked_list = []

		x_dec = 0.5
		y_dec = 0.5
		blocked_list.append((x-x_dec, y))
		blocked_list.append((x+x_dec, y))
		blocked_list.append((x, y-y_dec))
		blocked_list.append((x, y+y_dec))
		return blocked_list


	# populates the environment and a dictionary for the internal representation
	# of objects
	def generate_blocked_edges(self, houses, streets, packages, seed, root_path):
		object_dict = {}
		np.random.seed(seed)
		f_out = self.copy_empty_world(root_path)

		object_dict['packages'] = []

		for i in range(0, packages):
			box = Package(i, i*1.0, -2.0, random.randint(1, 4) / 4.0)
			self.add_box(f_out, box)
			object_dict['packages'].append(box)

		object_dict['houses'] = []
		object_dict['streets'] = []
		for i in range(0, streets):
			object_dict['streets'].append([])
			for j in range(0, houses):
				house = House(i*houses+j, i*5.0-5.0, j*5.0+5.0)
				self.add_house(f_out, house)
				object_dict['streets'][i].append(house)
				object_dict['houses'].append(house)

		f_out.write('</state>')

		for box in object_dict['packages']:
			self.add_box_description(f_out, box)
		
		for house in object_dict['houses']:
			self.add_house_description(f_out, house)
			
		f_out.write('</world>\n</sdf>')
		f_out.close()

		return object_dict
	

	def generate_launch(self, robots, objects, root_path):
		f_out = open(root_path+'/launch/neighborhood.launch', 'w')
		f_out.write('''<launch>
				<arg name="model" default="waffle" doc="model type [burger, waffle, waffle_pi]"/>
				<param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro"/>
				<include file="$(find gazebo_ros)/launch/empty_world.launch">
					<arg name="world_name" value="$(find group_16)/worlds/neighborhood.sdf"/>
					<arg name="paused" value="false"/>
					<arg name="use_sim_time" value="true"/>
					<arg name="gui" value="true"/>
					<arg name="headless" value="false"/>
					<arg name="debug" value="false"/>
				</include>\n''')
		
		objects['robots'] = []

		for i in range(0, robots):
			f_out.write('''
				<group ns="robot{0}">
					<param name="tf_prefix" value="robot{0}_tf" />
					<include file="$(find group_16)/launch/one_robot.launch" >
						<arg name="init_pose" value="-x {1} -y -0 -z 0" />
						<arg name="robot_name"  value="Robot{0}" />
					</include>
				</group>\n'''.format(i, i*2.0))
			objects['robots'].append(('robot'+str(i),i*2.0,0.0))
			
		f_out.write('</launch>\n')
		return objects

