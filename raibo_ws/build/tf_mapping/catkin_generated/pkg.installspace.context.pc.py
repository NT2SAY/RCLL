# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;tf;std_msgs;geometry_msgs;visualization_msgs;sensor_msgs;tf2_msgs;aruco".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltf_mapping".split(';') if "-ltf_mapping" != "" else []
PROJECT_NAME = "tf_mapping"
PROJECT_SPACE_DIR = "/home/rai/raibocup/raibo_ws/install"
PROJECT_VERSION = "1.0.0"
