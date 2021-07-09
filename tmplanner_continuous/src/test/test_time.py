import sched, time
import roslaunch
import rospy
from time import sleep
import subprocess
import os
import signal
import rospkg


s = sched.scheduler(time.time, time.sleep)


def plan_IPP():
     s.enter(4*60*60, 1, do_stuff, ())
     s.run()

def do_stuff(): 
	p = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/cov_traces.dat","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/aIPOP_cov_traces.dat"], stdout=subprocess.PIPE)
	p1 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/matrices/time_trace_traj.dat","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/aIPOP_time_trace_traj.dat"], stdout=subprocess.PIPE)

	print p.communicate()
	print p1.communicate()

	sleep(5)
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
		launch_tmp = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/launch/tmplanner_random_sampling.launch"])

		launch_mon.start()
		sleep(10)
		launch_tmp.start()
		sleep(15)

		p2 = subprocess.Popen(["rosservice call /firefly/start_planning "], stdout=subprocess.PIPE,stderr = subprocess.PIPE,shell=True)
		print p2.communicate()[0]
		print "sleeping"
		sleep(300)
		launch_mon.shutdown()
		launch_tmp.shutdown()
		sleep(15)

	p2 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/cov_traces.dat","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/random_sampling_cov_traces.dat"], stdout=subprocess.PIPE)
	p3 = subprocess.Popen(["cp","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/matrices/time_trace_traj.dat","/home/ajith/catkin_ws/src/tmplanner/tmplanner_continuous/debug/dataset/random_sampling_time_trace_traj.dat"], stdout=subprocess.PIPE)

	print p2.communicate()
	print p3.communicate()

	sleep(5)


plan_IPP()
