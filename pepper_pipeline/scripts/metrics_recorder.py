#!/usr/bin/env python3

import csv
from datetime import datetime

import rospy
from std_msgs.msg import Float64, Float32
import numpy as np
import time
from sfm_diff_drive.msg import SFMDriveActionResult, SFMDriveActionFeedback


def import_csv(csvfilename):
    """opens and return all content from csv in an array"""
    data = []
    with open(csvfilename, "r", encoding="utf-8", errors="ignore") as scraped:
        reader = csv.reader(scraped, delimiter=",")
        row_index = 0
        for row in reader:
            if row:  # avoid blank lines
                row_index += 1
                columns = [
                    str(row_index),
                    row[0],
                    row[1],
                    row[2],
                    row[3],
                    row[4],
                    row[5],
                ]
                data.append(columns)
        scraped.close()
    return data


class MetricsRecorder:
    """This class manages the state of the agents based on it position and time"""

    def save_value_csv(self):
        print("About to save data")
        """saves value of the metrics recorded in a csv"""
        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

        last_data = None
        try:
            csv_read_data = import_csv(
                self.csv_dir + self.solution_type + "/" + self.csv_name
            )
            last_data = csv_read_data[-1]
        except Exception as _e:
            pass
            # print(_e)

        print("csv imported")

        with open(
            self.csv_dir + self.solution_type + "/" + self.csv_name, "a", newline=""
        ) as csvfile_write, open(
            self.csv_dir + self.solution_type + "/" + self.csv_name,
            "r",
        ) as csvfile_read:
            reader = csv.reader(csvfile_read)
            fieldnames = [
                "test_number",
                "time",
                "goal_reached",
                "average_sii",
                "average_rmi",
                "total_time",
                "average_cpu",
            ]
            writer = csv.DictWriter(csvfile_write, fieldnames=fieldnames)
            try:
                if next(reader) != [
                    "test_number",
                    "time",
                    "goal_reached",
                    "average_sii",
                    "average_rmi",
                    "total_time",
                    "average_cpu",
                ]:
                    writer.writeheader()
            except:
                writer.writeheader()

            print("goal_reached: ", self.goal_reached)

            if last_data is not None:
                writer.writerow(
                    {
                        "test_number": int(last_data[1]) + 1,
                        "time": dt_string,
                        "goal_reached": self.goal_reached,
                        "average_sii": np.average(self.sii),
                        "average_rmi": np.average(self.rmi),
                        "total_time": self.total_time,
                        "average_cpu": np.average(self.cpu),
                    }
                )
            else:
                writer.writerow(
                    {
                        "test_number": 1,
                        "time": dt_string,
                        "goal_reached": self.goal_reached,
                        "average_sii": np.average(self.sii),
                        "average_rmi": np.average(self.rmi),
                        "total_time": self.total_time,
                        "average_cpu": np.average(self.cpu),
                    }
                )
            print("[INFO] [" + str(rospy.get_time()) + "] metrics for test saved.")
            csvfile_write.close()
            csvfile_read.close()

    def __init__(self):

        rospy.init_node("metrics_csv_writer", anonymous=True)

        rospy.on_shutdown(self.save_value_csv)

        # arrays for the metrics values to be stored

        self.rmi = np.array([], dtype=np.float64)

        self.sii = np.array([], dtype=np.float64)

        self.rmi = np.array([], dtype=np.float64)

        self.goal_reached = 0

        self.goal_available = False

        self.total_time = 0.0

        self.cpu = np.array([], dtype=np.float64)

        # ! directory of csv files
        self.csv_dir = rospy.get_param(
            "/frozen_agent_csv/csv_dir",
            "/home/sasm/people_sim_ws/src/pedsim_ros/pedsim_simulator/metrics/",
        )

        self.solution_type = rospy.get_param(
            "/frozen_agent_csv/solution_type", "position/"
        )

        self.csv_name = rospy.get_param("/frozen_agent_csv/csv_name", "first_test.csv")

        #! subcribers

        rospy.Subscriber(
            "/rmi",
            Float64,
            self.rmi_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            "/sii",
            Float64,
            self.sii_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            "/cpu_monitor/sfm_drive_node/cpu",
            Float32,
            self.cpu_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            "/sfm_drive_node/result",
            SFMDriveActionResult,
            self.goal_reached_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            "/sfm_drive_node/feedback",
            SFMDriveActionFeedback,
            self.goal_available_callback,
            queue_size=1,
        )

    # rmi metrics record
    def rmi_callback(self, msg):
        if self.goal_available:
            self.rmi = np.append(self.rmi, msg.data)

    # sii metrics record
    def sii_callback(self, msg):
        if self.goal_available:
            self.sii = np.append(self.sii, msg.data)

    # sfm goal reached callback
    def goal_reached_callback(self, msg):
        if msg.result.result == "waypoint reached":
            self.goal_reached = 1
            self.total_time = abs(time.time() - self.total_time)

    def goal_available_callback(self, msg):
        if msg.feedback.feedback == "robot moving":
            if not self.goal_available:
                self.goal_available = True
                self.total_time = time.time()

    def cpu_callback(self, msg):
        if self.goal_available:
            self.cpu = np.append(self.cpu, msg.data)


if __name__ == "__main__":
    csv_counter_saver = MetricsRecorder()
    while True:
        rospy.spin()
