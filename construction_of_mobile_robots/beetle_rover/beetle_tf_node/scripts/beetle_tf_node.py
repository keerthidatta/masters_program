import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def zed_odom_callback(camera_odom):
    #new_message = camera_odom
    new_message.pose.pose.position.z = new_message.pose.pose.position.z - 0.01857 - 0.45477 - 0.34254
    pub.Publish(new_message)

if __name__ == '__main__':
    rospy.init_node('beetle_tf_node', anonymous=True)
    new_message = Odometry()	
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('/zed/odom', Odometry, zed_odom_callback)
        pub = rospy.Publisher('/beetle_camera/camera_odom', Odometry, queue_size=100)

   
        
