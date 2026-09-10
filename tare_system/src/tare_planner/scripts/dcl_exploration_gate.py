#!/usr/bin/env python3

import string

import rospy
from std_msgs.msg import Bool


class DclExplorationGate:
    """Start local TARE only after every robot reports stable global DCL data."""

    def __init__(self):
        robot_num = rospy.get_param("~robot_num", 2)
        prefixes = rospy.get_param("~robot_prefixes", list(string.ascii_lowercase[:robot_num]))
        if len(prefixes) != robot_num:
            raise rospy.ROSException(
                "robot_prefixes length ({}) does not match robot_num ({})".format(
                    len(prefixes), robot_num
                )
            )

        self._stable_duration = rospy.Duration(rospy.get_param("~stable_duration", 2.0))
        self._ready = {prefix: False for prefix in prefixes}
        self._all_ready_since = None
        self._started = False
        self._start_pub = rospy.Publisher(
            rospy.get_param("~start_topic", "/start_exploration"), Bool, queue_size=1, latch=True
        )
        self._subscribers = []

        for prefix in prefixes:
            topic = "/{}/dcl_slam/alignment_ready".format(prefix)
            self._subscribers.append(
                rospy.Subscriber(topic, Bool, self._ready_callback, callback_args=prefix, queue_size=1)
            )
            rospy.loginfo("Waiting for DCL readiness topic: %s", topic)

        self._timer = rospy.Timer(rospy.Duration(0.1), self._timer_callback)

    def _ready_callback(self, message, prefix):
        self._ready[prefix] = message.data
        if not all(self._ready.values()):
            self._all_ready_since = None

    def _timer_callback(self, _event):
        if self._started or not all(self._ready.values()):
            return

        now = rospy.Time.now()
        if self._all_ready_since is None:
            self._all_ready_since = now
            return

        if now - self._all_ready_since < self._stable_duration:
            return

        self._start_pub.publish(Bool(data=True))
        self._started = True
        rospy.loginfo(
            "All %d DCL global alignments are ready; exploration started",
            len(self._ready),
        )


if __name__ == "__main__":
    rospy.init_node("dcl_exploration_gate")
    DclExplorationGate()
    rospy.spin()
