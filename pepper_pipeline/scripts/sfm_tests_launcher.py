import time
import roslaunch
import rospy
from sfm_diff_drive.msg import SFMDriveActionResult


PROCESS_GENERATE_RUNNING = True


class ProcessListener(roslaunch.pmon.ProcessListener):
    """keeps track of the process from launch file"""

    global PROCESS_GENERATE_RUNNING

    def process_died(self, name, exit_code):
        global PROCESS_GENERATE_RUNNING
        PROCESS_GENERATE_RUNNING = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def init_launch(launchfile, process_listener):
    """initiates launch file runs it"""
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launchfile],
        process_listeners=[process_listener],
    )
    return launch


rospy.init_node("sfm_tests_launcher")

LAUNCH_FILE = (
    "/home/sasm/ros/melodic/system/src/pepper_pipeline/launch/sfm_tests.launch"
)
launch = init_launch(LAUNCH_FILE, ProcessListener())
launch.start()

goal_reached = False


def goal_reached_callback(msg):
    global goal_reached
    if msg.result.result == "waypoint reached":
        goal_reached = True


rospy.Subscriber(
    "/sfm_drive_node/result",
    SFMDriveActionResult,
    goal_reached_callback,
    queue_size=1,
)

init_time = time.time()

while (time.time() - init_time < 800) and (not goal_reached):
    rospy.sleep(0.05)

launch.shutdown()
