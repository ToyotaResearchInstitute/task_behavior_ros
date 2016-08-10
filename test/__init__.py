import rospy


def setup_package():
    rospy.init_node('test')


def teardown_package():
    rospy.signal_shutdown('shutting down')
