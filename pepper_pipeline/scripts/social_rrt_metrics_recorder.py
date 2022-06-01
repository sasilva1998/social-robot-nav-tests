#!/usr/bin/env python3

import csv
from datetime import datetime

import rospy
from std_msgs.msg import Float64, Float32, Int32
import numpy as np
import time
from sfm_diff_drive.msg import SFMDriveActionResult, SFMDriveActionFeedback
from pepper_move_base_msgs.msg import Goto2DActionGoal, Goto2DActionResult


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
                    row[6],
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
        print(self.csv_dir + self.solution_type + "/" + self.csv_name)
        print(last_data)

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
                "collision_counter",
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
                    "collision_counter",
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
                        "average_sii": round(np.average(self.sii), 2),
                        "average_rmi": round(np.average(self.rmi), 2),
                        "total_time": self.total_time,
                        "average_cpu": round(np.average(self.cpu), 2),
                        "collision_counter": self.collision_counter,
                    }
                )
            else:
                writer.writerow(
                    {
                        "test_number": 1,
                        "time": dt_string,
                        "goal_reached": self.goal_reached,
                        "average_sii": round(np.average(self.sii), 2),
                        "average_rmi": round(np.average(self.rmi), 2),
                        "total_time": self.total_time,
                        "average_cpu": round(np.average(self.cpu), 2),
                        "collision_counter": self.collision_counter,
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

        self.collision_counter = 0

        # ! directory of csv files
        self.csv_dir = rospy.get_param("~csv_dir")

        self.solution_type = rospy.get_param("~solution_type")

        self.csv_name = rospy.get_param("~csv_name")

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
            "/cpu_monitor/pepper_move_base_planner/cpu",
            Float32,
            self.cpu_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            "/pepper_goto_action/result",
            Goto2DActionResult,
            self.goal_reached_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            "/pepper_goto_action/goal",
            Goto2DActionGoal,
            self.goal_available_callback,
            queue_size=1,
        )

        rospy.Subscriber(
            "/collision_counter",
            Int32,
            self.collision_counter_callback,
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
        if msg.result.success:
            self.goal_reached = 1
            self.total_time = abs(time.time() - self.total_time)

    def goal_available_callback(self, msg):
        if msg.goal:
            if not self.goal_available:
                self.goal_available = True
                self.total_time = time.time()

    def cpu_callback(self, msg):
        if self.goal_available:
            self.cpu = np.append(self.cpu, msg.data)

    def collision_counter_callback(self, msg):
        self.collision_counter = msg.data


if __name__ == "__main__":
    csv_counter_saver = MetricsRecorder()
    while True:
        rospy.spin()
