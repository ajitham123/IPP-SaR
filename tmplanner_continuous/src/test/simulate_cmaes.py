import roslaunch
import rospy
from time import sleep
import subprocess
import os
import signal
import rospkg

# rospy.init_node('my_node', anonymous=True)

# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
# launch_mon = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/launch/monitoring_example.launch"])
# launch_tmp = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/launch/tmplanner.launch"])

# launch_mon.start()
# sleep(10)
# launch_tmp.start()
# sleep(15)

# p2 = subprocess.Popen(["rosservice call /firefly/start_planning "], stdout=subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
# print p2.communicate()[0]
# print "sleeping"
# sleep(300)
# launch_mon.shutdown()
# launch_tmp.shutdown()
# sleep(15)

# rewrite the file to a blank file
rospack = rospkg.RosPack()
file = open(rospack.get_path('tmplanner_continuous') + "/debug/cov_traces.dat","w") 
file.close()

file1 = open(rospack.get_path('tmplanner_continuous') + "/debug/time_trace_traj.dat","w") 
file1.close()

# p1 = subprocess.Popen(["catkin build tmplanner_continuous  --no-deps"], stdout=subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
# print p1.communicate()[0]
# sleep(100)

for i in range(25):
	print "\n\n\n\n\n\n\n Running trial number: "+str(i+1)
	# launch_gazebo = subprocess.Popen("roslaunch tmplanner_continuous monitoring_example.launch".split(), stdout=subprocess.PIPE)
	# sleep(10)
	# launch_planner = subprocess.Popen("roslaunch tmplanner_continuous tmplanner.launch".split(), stdout=subprocess.PIPE)
	# sleep(15)

	# # p = subprocess.Popen(["roslaunch tmplanner_continuous monitoring_example.launch "],stdout=subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
	# call_service = subprocess.Popen(["rosservice call /firefly/start_planning "], stdout=subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
	# print call_service.communicate()[0]
	# sleep(300)

	# # os.killpg(os.getpgid(launch_gazebo.pid), signal.SIGTERM)
	# # os.killpg(os.getpgid(launch_planner.pid), signal.SIGTERM)
	# launch_gazebo.terminate()
	# launch_planner.terminate()
	# sleep(15)


	rospy.init_node('my_node', anonymous=True)

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	launch_mon = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/launch/monitoring_example.launch"])
	launch_tmp = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/launch/tmplanner.launch"])

	launch_mon.start()
	sleep(10)
	launch_tmp.start()
	sleep(20)

	p2 = subprocess.Popen(["rosservice call /firefly/start_planning "], stdout=subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
	print p2.communicate()[0]
	print "sleeping"
	sleep(610)
	launch_mon.shutdown()
	launch_tmp.shutdown()
	sleep(15)
# p2 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/cov_traces.dat",\
# 	"/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/random_boxes/CMAES_hard_15RandBox_small_buil_cov_traces.dat"], stdout=subprocess.PIPE)
# p3 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/time_trace_traj.dat",\
# 	"/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/random_boxes/CMAES_hard_15RandBox_small_buil_time_trace_traj.dat"], stdout=subprocess.PIPE)
p4 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/metric.dat",\
	"/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/coverage_RMSE/aBIPOP_CMAES_hard_covariance_RMSE.dat"], stdout=subprocess.PIPE)
p5 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/metric_traj.dat",\
	"/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/coverage_RMSE/aBIPOP_CMAES_hard_covariance_RMSE_traj.dat"], stdout=subprocess.PIPE)
# print p2.communicate()
# print p3.communicate()
print p4.communicate()
print p5.communicate()

sleep(10)
