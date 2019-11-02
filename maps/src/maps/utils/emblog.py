""" Lightweight logger to change logging environment depending on whether ros is available. """

try:
    import rospy
    import rostopic
    try:
        rostopic.get_topic_class('/rosout')
        USE_ROS = True
    except rostopic.ROSTopicIOException:
        USE_ROS = False
except ImportError:
    USE_ROS = False


class Color(object):
    DEBUG = '\033[92m'
    WARN = '\033[93m'
    ERROR = '\033[91m'
    ENDC = '\033[0m'


def set_ros(use_ros):
    """ Use this to set the output to be ros out or std out. """
    global USE_ROS
    USE_ROS = use_ros


def debug(msg, *args, **kwargs):
    if USE_ROS:
        rospy.logdebug(msg, *args)
    else:
        print Color.DEBUG + msg + Color.ENDC


def info(msg, *args, **kwargs):
    if USE_ROS:
        rospy.loginfo(msg, *args)
    else:
        print msg


def warn(msg, *args, **kwargs):
    if USE_ROS:
        rospy.logwarn(msg, *args)
    else:
        print Color.WARN + msg + Color.ENDC


def error(msg, *args, **kwargs):
    if USE_ROS:
        rospy.logerr(msg, *args)
    else:
        print Color.ERROR + msg + Color.ENDC
