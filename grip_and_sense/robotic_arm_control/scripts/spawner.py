#! /usr/bin/env python3

import rospy
import random
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import numpy as np


class model_spawner():

  def __init__(self):
      self.poses = [(0.65,0.65,0.1), (-0.65,-0.65,0.2), (0.5,-0.5,0.35), (-0.65,0.65,0.3), (0.35,0.35,0.2), (-0.45,-0.45,0.1),
                    (0.8,-0.8,0.25), (-0.8,0.8,0.2), (0.95,0.95,0.1), (-0.95,-0.55,0.1)]
      self.points = []
      self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
      self.model_xml_header = ''' <sdf version='1.7'>\n<model name=" '''
      self.model_xml_body = '''">
      <link name="link">
        <pose>0 0 0.1 0 0 0</pose>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.2</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.2</size>
            </box>
          </geometry>
        </visual>
        <sensor name='my_contact' type='contact'>
        <plugin name='cont_plug' filename='libcontact.so'/>
          <contact>
            <collision>box_collision</collision>
            <update_rate>5</update_rate>
          </contact>
        </sensor>
       
      </link>
        <static>1</static>
      </model>
    </sdf>'''

  def get_random_pose(self): 
    index = random.randint(0, len(self.poses) - 1)
    pose = self.poses[index]
    self.poses.pop(index)
    return pose

  def spawn_contact_model(self, box_name):
    model_name = box_name
    model_xml = str(self.model_xml_header) + str(box_name) + str(self.model_xml_body)
    initial_pose = Pose()
    spawn_pose = self.get_random_pose()
    initial_pose.position.x = spawn_pose[0]
    initial_pose.position.y = spawn_pose[1]
    initial_pose.position.z = spawn_pose[2]
    self.points.append(self.transformPoint(spawn_pose))

    self.spawn_model.call(model_name, model_xml, "", initial_pose, "world")
  
  def get_points_to_reach(self):
    return self.points
  
  def transformPoint(self,point):
    try:
      aPborg = np.array([0.1, 0.0, -0.2])
      return np.array(point) + aPborg
    except Exception as e:
      print(e)
      pass
