from os import terminal_size
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

public_velo = rospy.Publisher("/cmd_vel",Twist, queue_size=5)
my_Velo = Twist()

pic_index=1
CSR = 3
ROS = 25


class List_Point:
	def __init__(self):
		self.row = np.array(range(20))
		self.col = np.array(range(20))
		self.trust = np.array(range(20))
		self.size=0

def callbackFunction(image):
	bridge = CvBridge()
	global pic_index
	pic = bridge.imgmsg_to_cv2(image, image.encoding)
	print(str(pic_index)+' => running normally!')
	pic=cv2.resize(pic,dsize=(640,480))
	pic=pic[240:480,0:640]
	pic=cv2.cvtColor(pic,cv2.COLOR_BGR2GRAY)
	pts1=np.float32([[500,20],[640,150],[0,150],[140,20]])
	pts2=np.float32([[768,0],[512,512],[256,512],[0,0]])

	matrix = cv2.getPerspectiveTransform(pts1,pts2)
	frame=cv2.warpPerspective(pic,matrix,(768,512))
	frame = cv2.adaptiveThreshold(frame, 255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 5)

	for x in range(0,768):
		for y in range(0,512):
			if (1.04*x+246 - y < 0) or (-0.922*x-y+980 < 0):
				frame[y][x]=0

	frame_for_draw = np.copy(frame)
	frame_for_draw = cv2.cvtColor(frame_for_draw, cv2.COLOR_GRAY2BGR)

	left=List_Point()
	right=List_Point()
	center=List_Point()
	trust_list=List_Point()
	initial_row = 500
	initial_col = 384
	count = 0

	cutting_pic = frame[300:512, 180:580]
	sum_of_cut_pic = np.array(range(0, 400))
	for i in range(0, 400):
		sum_of_cut_pic[i] = sum(cutting_pic[:, i])

	max_left = 0
	max_left_index = 0
	max_right = 0
	max_right_index = 0
	tmp_pre = sum_of_cut_pic[200]
	for i in range(201, 400):
		if (sum_of_cut_pic[i] > 10000):
			max_right = sum_of_cut_pic[i]
			max_right_index = i
			break
	frame_for_draw = cv2.line(frame_for_draw, (max_right_index + 180, 0), (max_right_index + 180, 512), (255, 0, 0), 2)
	tmp_pre = sum_of_cut_pic[199]
	for i in range(200,0,-1):
		if (sum_of_cut_pic[i] > 10000):
			max_left = sum_of_cut_pic[i]
			max_left_index = i
			break
	frame_for_draw = cv2.line(frame_for_draw, (max_left_index + 180, 0), (max_left_index + 180, 512), (255, 0, 0), 2)

	max_left_index += 180
	max_right_index += 180

	for i in range(0, 20):
		index = (i + 30) * 10
		for slide in range(max_left_index + ROS, max_left_index - ROS, -1):
			if not (frame[index][slide] == 0):
				break
		left.row[i] = index
		left.col[i] = slide
		if slide == max_left_index - ROS + 1:
			left.trust[i] = 0
		else:
			left.trust[i] = 1

		for slide in range(max_right_index - ROS, max_right_index + ROS):
			if not (frame[index][slide] == 0):
				break
		right.row[i] = index
		right.col[i] = slide
		if slide == max_right_index + ROS - 1:
			right.trust[i] = 0
		else:
			right.trust[i] = 1

	for i in range(0, 20):
		center.col[i] = (left.col[i] + right.col[i]) // 2
		center.row[i] = left.row[i]
		if left.trust[i] and right.trust[i]:
			center.trust[i] = 1
		else:
			center.trust[i] = 0

	for i in range(1,19):
		tmp_value = (center.col[i+1] + center.col[i-1])/2 - center.col[i]
		if tmp_value*tmp_value > CSR:
			center.trust[i]=0

	for i in range(0, 20):
		if center.trust[i]:
			#if (linear_predict[i] < 5 and linear_predict[i] > -5):
			frame_for_draw = cv2.rectangle(frame_for_draw, (center.col[i] + 3, center.row[i] + 1),
			                               (center.col[i] - 3, center.row[i] - 1), (0,255,0), 2)
			#else:
			 #   frame_for_draw = cv2.rectangle(frame_for_draw, (center.col[i] + 3, center.row[i] + 1),
			  #                                 (center.col[i] - 3, center.row[i] - 1), (255, 0, 255), 2)
			trust_list.row[trust_list.size] = center.row[i]
			trust_list.col[trust_list.size] = center.col[i]
			trust_list.size += 1
		# else:
		# frame_for_draw = cv2.rectangle(frame_for_draw, (center.col[i] + 3, center.row[i] + 1),
		#                              (center.col[i] - 3, center.row[i] - 1), (0,255,0), 1)
	final_index=0
	global pre_velo
	final_velo = 0
	if trust_list.size > 5:
		print("road detected !")
		
		for i in range(trust_list.size-1,trust_list.size-6,-1):
			final_index+=trust_list.col[i]
		final_index=final_index/5
		final_velo = ((384 - final_index)/1000)
		pre_velo=final_velo
	else :
		print("road undetected !")
		final_velo = pre_velo

	cv2.imshow('output',frame_for_draw)
	global public_velo
	global my_Velo
	
	my_Velo.linear.x=0.1
	my_Velo.angular.z=final_velo
	print('publish: ',final_velo)

	if cv2.waitKey(1) == 27:
		print("EXIT")

	pic_index+=1
	public_velo.publish(my_Velo)


while not rospy.is_shutdown():
	rospy.init_node("topic_receiver",anonymous=True)
	lis_scan = rospy.Subscriber("/camera/rgb/image_raw", Image, callback=callbackFunction)
	rospy.spin()