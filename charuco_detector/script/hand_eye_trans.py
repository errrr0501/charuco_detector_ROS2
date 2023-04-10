#!/usr/bin/env python3
import os
import numpy as np
import rospy
import rospkg
import tf
import configparser
from math import asin, atan2, degrees, pi
from charuco_detector.srv import eye2base, eye2baseResponse
from visp_hand2eye_calibration.msg import TransformArray
from geometry_msgs.msg import Transform



class HandEyeTrans:
	def __init__(self, base_link, tip_link, eye_in_hand_mode, customize, filename):
		self.base_link = base_link
		self.tip_link = tip_link
		self.eye_in_hand_mode = eye_in_hand_mode
		self.customize = customize
		self.filename = filename
		self.save_camera_cali_pwd = os.path.join(os.path.dirname(__file__), '..','config/','camera_calibration/')
		self.hand_eye_cali_pwd = os.path.join(os.path.dirname(__file__), '..','config/','hand_eye_calibration/')
		self.tf_listener = tf.TransformListener()
		self.base_h_tool = TransformArray()
		self.camera_h_charuco = TransformArray()
		self.base_h_tool.header.frame_id = self.base_link
		self.camera_h_charuco.header.frame_id = 'calib_camera'
		self._rtool_tool_trans = np.mat(np.identity(4))

		self._img_pose = np.zeros(6)
		self._base_pose = np.zeros(6)
		self._curr_pose = np.zeros(6)
		self._tool_coor = np.zeros(6)
		self._base_coor = np.zeros(6)
		self._base_tool_trans =  np.mat(np.identity(4))
		self._hand_eye_trans =   np.mat(np.identity(4))
		self._rtool_eye_trans, self._camera_mat = self.__get_camera_param()

		self.__eye2base_server = rospy.Service('eye2base',
				eye2base,
				self.__eye2base_transform
		)
		self.__pix2base_server = rospy.Service('pix2base',
				eye2base,
				self.__pix2base_transform
		)
		self.__eye_trans2base_server = rospy.Service('eye_trans2base',
				eye2base,
				self.__eye_trans2base_transform
		)


	def __get_camera_param(self):
		config = configparser.ConfigParser()
		config.optionxform = str

		config.read(self.save_camera_cali_pwd + 'camera_calibration.ini')
		
		K00 = float(config.get("Intrinsic", "0_0"))
		K01 = float(config.get("Intrinsic", "0_1"))
		K02 = float(config.get("Intrinsic", "0_2"))
		K10 = float(config.get("Intrinsic", "1_0"))
		K11 = float(config.get("Intrinsic", "1_1"))
		K12 = float(config.get("Intrinsic", "1_2"))
		K20 = float(config.get("Intrinsic", "2_0"))
		K21 = float(config.get("Intrinsic", "2_1"))
		K22 = float(config.get("Intrinsic", "2_2"))

		if self.customize:
			config.read(self.hand_eye_cali_pwd + self.filename)
		else:
			if self.eye_in_hand_mode:
				config.read(self.hand_eye_cali_pwd + 'eye_in_hand_calibration.ini')
			else:
				config.read(self.hand_eye_cali_pwd + 'eye_to_hand_calibration.ini')

		x  = float(config.get("hand_eye_calibration", "x" ))
		y  = float(config.get("hand_eye_calibration", "y" ))
		z  = float(config.get("hand_eye_calibration", "z" ))
		qx = float(config.get("hand_eye_calibration", "qx"))
		qy = float(config.get("hand_eye_calibration", "qy"))
		qz = float(config.get("hand_eye_calibration", "qz"))
		qw = float(config.get("hand_eye_calibration", "qw"))



		tool_eye_trans = self.tf_listener.fromTranslationRotation((x,y,z),(qx,qy,qz,qw))
		# print("tool_eye_trans",tool_eye_trans)


		Intrinsic = np.mat([[K00, K01, K02],
					 		[K10, K11, K12],
					 		[K20, K21, K22]])
		return tool_eye_trans, Intrinsic

	def __get_robot_trans(self):
		try:
			(base_trans_tool, base_rot_tool) = self.tf_listener.lookupTransform(self.base_link, self.tip_link, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logwarn('lookupTransform for robot failed!, ' + self.base_link + ', ' + self.tip_link)

		trans = Transform()
		trans.translation.x = base_trans_tool[0]
		trans.translation.y = base_trans_tool[1]
		trans.translation.z = base_trans_tool[2]
		trans.rotation.x = base_rot_tool[0]
		trans.rotation.y = base_rot_tool[1]
		trans.rotation.z = base_rot_tool[2]
		trans.rotation.w = base_rot_tool[3]
		self.base_h_tool.transforms.append(trans)

		if self.eye_in_hand_mode:
			self._base_tool_trans = self.tf_listener.fromTranslationRotation((trans.translation.x,trans.translation.y,trans.translation.z),
																			(trans.rotation.x,trans.rotation.y,trans.rotation.z,trans.rotation.w))
		else:
			pass
		return

	def __rotation2quat(self, rotation):

		#################################################################################################
		quat_result = tf.transformations.quaternion_from_matrix(rotation)
		####################################################################################################

		# print("---------------------------------")
		# print(quat_result)
		return quat_result
		
	def __eye2base_transform(self, req):
		##################not finish yet############################
		self.__get_robot_trans()
		assert len(req.ini_pose) == 3
		eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
		# eye_obj_trans[:3] = np.multiply(eye_obj_trans[:3], 0.01)
		result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
		print(result)
		res = eye2baseResponse()
		res.trans = np.mat(np.identity(4))
		res.trans[2, :] = result.flatten()
		print(res.trans)
		res.trans = np.squeeze(np.asarray(res.trans))
		# res.trans = np.array(res.trans.T)
		
		res.pos = np.array(result[:3]).reshape(-1)
		res.quat = self.__rotation2quat(res.trans)
		res.trans = []
		return res

	def __eye_trans2base_transform(self, req):
		self.__get_robot_trans()
		assert len(req.ini_pose) == 16
		eye_obj_trans = np.mat(req.ini_pose).reshape(4, 4)
		result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
		res = eye2baseResponse()
		res.trans = np.array(result).reshape(-1)
		res.pos = np.array(result[0:3, 3]).reshape(-1)
		res.quat = self.__rotation2quat(result)
		# res.quat = self.__rotation2quat(result[0:3,0:3])
		return res
		

	def __pix2base_transform(self, req):
		##################not finish yet############################
		self.__get_robot_trans()
		assert len(req.ini_pose) == 3
		eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
		# eye_obj_trans[2] = eye_obj_trans[2] * 0.01
		eye_obj_trans[:2] = (eye_obj_trans[:2] - self._camera_mat[:2, 2:]) * eye_obj_trans[2]
		eye_obj_trans[:2] = np.multiply(eye_obj_trans[:2], [[1/self._camera_mat[0, 0]], [1/self._camera_mat[1, 1]]])
		result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
		res = eye2baseResponse()
		res.trans = np.mat(np.identity(4))
		res.trans[2, :] = result
		res.pos = np.array(result[:3]).reshape(-1)
		res.quat = self.__rotation2quat(res.trans)
		return res

if __name__ == "__main__":
	rospy.init_node('hand_eye_trans')
	base_link = rospy.get_param('~base_link')
	tip_link = rospy.get_param('~tip_link')
	eye_in_hand_mode = rospy.get_param('~eye_in_hand_mode')
	customize = rospy.get_param('~customize')
	filename = rospy.get_param('~filename')
	processing = HandEyeTrans(base_link, tip_link, eye_in_hand_mode, customize, filename)
	rospy.spin()