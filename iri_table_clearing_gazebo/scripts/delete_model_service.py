#! /usr/bin/env python
# Author: Nicola Covallero
# This scripts remove a model from Gazebo given the coordinates of the model. 

import roslib # ; roslib.load_manifest('iri_wam_controller')
import rospy
import actionlib
import tf
import math
from iri_table_clearing_gazebo.srv import DeleteObject
from gazebo_msgs.srv import DeleteModel 
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import  PointStamped

global data
global gazebo_delete_srv_name

class Point:
  def __init__(self, x_,y_,z_):
    self.x = x_
    self.y = y_
    self.z = z_

def eucDist(p1,p2):
  return  math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def ModelStatesCallback(data_):
  global data  
  data = data_
      
def detectObject(req):
  global data
  global listener

  p1 = req.point  
  print p1
  p1.header.stamp = rospy.Time.now() # fake the time stamp
  p1 = listener.transformPoint('/world',p1)
  print p1
  
  # check the nearest object in gazebo
  model_name = None
  dist_min = 100.0
  counter = 0
  print req
  for o in data.pose:
    p_1 = Point(p1.point.x, p1.point.y, p1.point.z)
    p_2 = Point(o.position.x,o.position.y,o.position.z)
    dist =  eucDist(p_1,p_2)

    # compute minimum distance
    if dist_min > dist: 
      dist_min = dist
      model_name = data.name[counter]

    counter = counter + 1

  print 'Removing object: ', model_name
  print 'Minimum distance: ', dist_min

  res = gazebo_delete_srv(model_name)
  return res.success


################################

def service():
  global gazebo_delete_srv
  global listener

  rospy.init_node('delete_model_service')

  listener = tf.TransformListener()


  robot = rospy.get_param("~robot",default="estirabot")
  base_frame = "{}_link_base".format(robot)

  srv_name = "/{robot}/delete_model".format(robot=robot)
  gazebo_delete_srv_name = "/gazebo/delete_model"

  rospy.wait_for_service(gazebo_delete_srv_name)
  gazebo_delete_srv = rospy.ServiceProxy(gazebo_delete_srv_name, DeleteModel)

  delete_model_srv = rospy.Service(srv_name, DeleteObject, detectObject)

  rospy.Subscriber("/gazebo/model_states", ModelStates, ModelStatesCallback)
  
  rospy.spin()


if __name__ == '__main__':
  try:
    service()
  except rospy.ROSInterruptException:
    pass