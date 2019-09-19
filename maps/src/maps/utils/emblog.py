""" Lightweight logger to change logging environment depending on whether ros is available. """

try:
    import rospy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class Color(object):
    DEBUG = '\033[92m'
    WARN = '\033[93m'
    ERROR = '\033[91m'
    ENDC = '\033[0m'


def debug(msg, *args, **kwargs):
    if ROS_AVAILABLE:
        rospy.logdebug(msg, args, kwargs)
    print Color.DEBUG + msg + Color.ENDC


def info(msg, *args, **kwargs):
    if ROS_AVAILABLE:
        rospy.loginfo(msg, args, kwargs)
    print msg


def warn(msg, *args, **kwargs):
    if ROS_AVAILABLE:
        rospy.logwarn(msg, args, kwargs)
    print Color.WARN + msg + Color.ENDC


def error(msg, *args, **kwargs):
    if ROS_AVAILABLE:
        rospy.logerr(msg, args, kwargs)
    print Color.ERROR + msg + Color.ENDC
