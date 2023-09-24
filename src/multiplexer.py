#!/usr/bin/env python3

from threading import Lock

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from planner_interfacing.srv import SetPrecision, SetPrecisionRequest, SetPrecisionResponse

from planner_multiplexer.srv import SetPlanner, SetPlannerRequest, SetPlannerResponse

### main #####################################################################

def main():
	PlannerMultiplexer().loop()

class PlannerMultiplexer:
	def __init__(self) -> None:

		rospy.init_node('planner_multiplexer')

		### local variables ##################################################

		self.current_planner_lock = Lock()
		self.current_planner_name = rospy.get_param("~default_planner")
		self.current_planner_enabled = False
		self.current_planner_precision = rospy.get_param("~default_planner_precision")

		### connect to ROS ###################################################

		self.planner_status_topic = rospy.get_param("~planner_status_topic")
		self.select_planner_service = rospy.get_param("~select_planner_service")
		self.enabled_service = rospy.get_param("~enabled_service")
		self.precision_service = rospy.get_param("~precision_service")

		self.planner_status_pub = rospy.Publisher(self.planner_status_topic, String, queue_size=1)
		self.select_planner_srv = rospy.Service(self.select_planner_service, SetPlanner, self.select_planner_callback)
		self.enabled_srv = rospy.Service(self.enabled_service, SetBool, self.enabled_callback)
		self.precision_srv = rospy.Service(self.precision_service, SetPrecision, self.precision_callback)

		self.planner_names = rospy.get_param("~planner_names")
		self.planner_status_topics = rospy.get_param("~planner_status_topics")
		self.enabled_services = rospy.get_param("~enabled_services")
		self.precision_services = rospy.get_param("~precision_services")

		self.planner_status_subs = [
			rospy.Subscriber(
				topic, String,
				lambda status, planner=planner_name: self.planner_status_callback(planner, status)
			) for planner_name, topic in zip(self.planner_names, self.planner_status_topics)
		]
		self.enabled_srvs = [rospy.ServiceProxy(service, SetBool) for service in self.enabled_services]
		self.precision_srvs = [rospy.ServiceProxy(service, SetPrecision) for service in self.precision_services]

		### end init #########################################################

	### callbacks ############################################################

	def select_planner_callback(self, planner_req: SetPlannerRequest) -> SetPlannerResponse:
		new_planner_name = planner_req.planner

		with self.current_planner_lock:
			current_planner_name = self.current_planner_name
			current_enabled = self.current_planner_enabled
			current_precision = self.current_planner_precision

			self.current_planner_name = new_planner_name

		# prepare response
		response = SetPlannerResponse()
		response.success = True
		response.message = f"Selected {current_planner_name}"

		# get necessary service proxies
		try:
			current_planner_index = self.planner_names.index(current_planner_name)
			current_planner_enabled_srv = self.enabled_srvs[current_planner_index]

			new_planner_index = self.planner_names.index(new_planner_name)
			new_planner_enabled_srv = self.enabled_srvs[new_planner_index]
			new_planner_precision_srv = self.precision_srvs[new_planner_index]
		except ValueError:
			err_msg = f"Error finding services for {new_planner_name}"
			rospy.logerr(err_msg)
			response.success = True
			response.message = err_msg
			return response

		try:
			# switch out of current planner
			current_planner_enabled_srv(data=False)

			# switch into new planner
			new_planner_enabled_srv(data=current_enabled)
			new_planner_precision_srv(precision=current_precision)
		except rospy.service.ServiceException:
			err_msg = f"Error switching to {current_planner_name}"
			rospy.logerr(err_msg)
			response.success = True
			response.message = err_msg
			return response

		return response

	def enabled_callback(self, enabled_req: SetBoolRequest) -> SetBoolResponse:
		enable = enabled_req.data

		with self.current_planner_lock:
			current_planner_name = self.current_planner_name

		# prepare response
		response = SetBoolResponse()
		response.success = True
		response.message = f"Enabled {current_planner_name}"

		# get necessary service proxy
		try:
			current_planner_index = self.planner_names.index(current_planner_name)
			current_planner_enabled_srv = self.enabled_srvs[current_planner_index]
		except ValueError:
			err_msg = f"Error finding enable service for {current_planner_name}"
			rospy.logerr(err_msg)
			response.success = True
			response.message = err_msg
			return response

		try:
			current_planner_enabled_srv(data=enable)
			with self.current_planner_lock:
				self.current_planner_enabled = enable
		except rospy.service.ServiceException:
			err_msg = f"Error enabling {current_planner_name}"
			rospy.logerr(err_msg)
			response.success = True
			response.message = err_msg
			return response

		return response

	def precision_callback(self, precision_req: SetPrecisionRequest) -> SetPrecisionResponse:
		precision = precision_req.precision

		with self.current_planner_lock:
			current_planner_name = self.current_planner_name

		# prepare response
		response = SetPrecisionResponse()
		response.success = True
		response.message = f"Set precision for {current_planner_name}"

		# get necessary service proxy
		try:
			current_planner_index = self.planner_names.index(current_planner_name)
			current_planner_precision_srv = self.precision_srvs[current_planner_index]
		except ValueError:
			err_msg = f"Error setting precision of {current_planner_name}"
			rospy.logerr(err_msg)
			response.success = True
			response.message = err_msg
			return response

		try:
			current_planner_precision_srv(precision=precision)
			with self.current_planner_lock:
				self.current_planner_precision = precision
		except rospy.service.ServiceException:
			err_msg = f"Error setting precision of {current_planner_name}"
			rospy.logerr(err_msg)
			response.success = True
			response.message = err_msg
			return response

		return response
	
	def planner_status_callback(self, planner_name, status):

		with self.current_planner_lock:
			current_planner_name = self.current_planner_name

		if current_planner_name == planner_name:
			self.planner_status_pub.publish(status)

	### loop #################################################################

	def loop(self):
		rospy.spin()

if __name__ == '__main__':
	main()
