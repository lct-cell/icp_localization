import rospy
import csv
from nav_msgs.msg import Odometry
import tf

size = 396

myfile = open("/home/lct/nctu_sdc/localization_ws/submit2.csv", "w+")
csv_writer = csv.writer(myfile)
csv_writer.writerow(["id", "x", "y", "z", "yaw", "pitch", "roll"])

id_cnt = 1

def callback(odom):
	global id_cnt
	global csv_writer
	data = []
	data.append(id_cnt)
	data.extend([odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0])

	quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
	euler_angle = tf.transformations.euler_from_quaternion(quaternion)
	#euler_angle = [roll pitch yaw]
	data.extend([euler_angle[2], euler_angle[1], euler_angle[0]])

	csv_writer.writerow(data)

	print(id_cnt)
	id_cnt += 1

##------main------
if __name__ == '__main__':
	rospy.init_node("write_csv_node", anonymous=True)
	rospy.Subscriber("/result_localization", Odometry, callback)
	rospy.spin()
