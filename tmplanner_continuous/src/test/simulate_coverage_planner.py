import roslaunch
import rospy
from time import sleep
import subprocess
import os
import signal
import rospkg

# rewrite the file to a blank file
rospack = rospkg.RosPack()
file = open(rospack.get_path('tmplanner_continuous') + "/debug/cov_traces.dat","w") 
file.close()

file1 = open(rospack.get_path('tmplanner_continuous') + "/debug/matrices/time_trace_traj.dat","w") 
file1.close()

for i in range(100):
	print "\n\n\n\n\n\n\n Running trial number: "+str(i+1)


	rospy.init_node('my_node', anonymous=True)

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	launch_mon = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/launch/monitoring_example.launch"])
	launch_tmp = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/launch/tmplanner.launch"])
	launch_cov = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ajith/catkin_ws/src/mav_coverage_planning/launch/coverage_planner.launch"])

	launch_mon.start()
	sleep(10)
	launch_cov.start()
	sleep(3)
	launch_tmp.start()
	sleep(15)

	with open('/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/parameters/coverage_goals.yaml', 'r') as myfile:
		arguments = myfile.read()
		print arguments
	p1 = subprocess.Popen('rosservice call /mav_coverage_planning/plan_path '+arguments, stdout=subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
	print p1.communicate()[0]
	sleep(5)

	p2 = subprocess.Popen(["rosservice call /firefly/start_planning "], stdout=subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
	print p2.communicate()[0]

	sleep(300)
	launch_mon.shutdown()
	launch_tmp.shutdown()
	launch_cov.shutdown()
	sleep(15)
p2 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/cov_traces.dat","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/coverage_planning_cov_traces.dat"], stdout=subprocess.PIPE)
p3 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/matrices/time_trace_traj.dat","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/coverage_planning_time_trace_traj.dat"], stdout=subprocess.PIPE)
print p2.communicate()
print p3.communicate()

sleep(5)

# 	p1 = subprocess.Popen('rosservice call /mav_coverage_planning/plan_path  \
# \"start_pose: \n\
#  header: \n\
#   seq:0 \n\
#   stamp:{secs:0,nsecs:0}\n\
#   # frame_id:''\n\
#  pose: \n\
#   position:{x:-10.0,y:0.0,z:13.0} \n\
#   orientation:{x:0.0,y:0.0,z:0.0,w:0.0}\n\
# start_velocity:{x:0.0,y:0.0,z:0.0}\n\
# goal_pose:\n\
#  header: \n\
#   seq:0 \n\
#   stamp:{secs:0,nsecs:0}\n\
#   frame_id: ''\n\
#  pose: \n\
#   position:{x:-10.0,y:0.0,z:13.0} \n\
#   orientation:{x:0.0,y:0.0,z:0.0,w:0.0}\n\
# goal_velocity:{x:0.0,y:0.0,z:0.0}\n\
# bounding_box:{x:0.0,y:0.0,z:0.0}\"   ', stdout=subprocess.PIPE, shell=True)

	# p1 = subprocess.Popen('rosservice call /mav_coverage_planning/plan_path [start_pose.header.seq=0,start_pose.header.stamp=0,start_pose.header.frame_id='',start_pose.pose.position.x=-10,start_pose.pose.position.y=0,start_pose.pose.position.z=13,start_pose.pose.orientation.x=0,start_pose.pose.orientation.y=0,start_pose.pose.orientation.z=0,start_pose.pose.orientation.w=0,start_velocity.x=0,start_velocity.y=0,start_velocity.z=0,goal_pose.header.seq=0,goal_pose.header.stamp=0,goal_pose.header.frame_id='',goal_pose.pose.position.x=-10,goal_pose.pose.position.y=0,goal_pose.pose.position.z=13,goal_pose.pose.orientation.x=0,goal_pose.pose.orientation.y=0,goal_pose.pose.orientation.z=0,goal_pose.pose.orientation.w=0,goal_velocity.x=0,goal_velocity.y=0,goal_velocity.z=0,bounding_box.x=0,bounding_box.y=0,bounding_box.z=0]', stdout=subprocess.PIPE, shell=True)
	# p1 = subprocess.Popen('rosservice call /mav_coverage_planning/plan_path ', stdout=subprocess.PIPE, shell=True)

