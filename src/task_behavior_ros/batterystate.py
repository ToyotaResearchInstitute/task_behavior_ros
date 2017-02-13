# Copyright 2016 Toyota Research Institute

# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy
# of the License at

#   http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
import rospy

from sensor_msgs.msg import BatteryState

from task_behavior_engine.tree import NodeStatus
from task_behavior_ros.topic import TopicMonitor


class ChargeCompleteMonitor(TopicMonitor):

    """
    Checks if msg indicates charging is complete.
    @param name [str] The name of this node
    @param name [topic_name] The name of the topic
    @param queue_size [int] The size of the message queue

    NodeData:
        @param max_charge [float] Number between (0., 1.) that indicates charging percentage
    returns NodeStatus.SUCCESS if charging complete
    returns NodeStatus.FAIL if not charging or not complete
    """

    def __init__(self, name, topic_name, queue_size=1, *args, **kwargs):
        super(ChargeCompleteMonitor, self).__init__(
            name=name, topic_name=topic_name, topic_type=BatteryState,
            cb=self.charge_complete_cb, queue_size=1, *args, **kwargs)

    def charge_complete_cb(self, msg, nodedata):
        rospy.loginfo('received msg: ' + str(msg))
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL:
            return NodeStatus(NodeStatus.SUCCESS, "Power supply state full")
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING \
                and msg.percentage > nodedata.get_data('max_charge', 1.):
            return NodeStatus(NodeStatus.SUCCESS, "charge above max")
        return NodeStatus(NodeStatus.FAIL, "charge not complete")


class ChargeOKMonitor(TopicMonitor):

    """
    Checks if msg indicates charge is ok.
    @param name [str] The name of this node
    @param name [topic_name] The name of the topic
    @param queue_size [int] The size of the message queue

    NodeData:
       @param min_charge [float] Number between (0., 1.) that indicates minimum charging percentage
    returns NodeStatus.SUCCESS if charge ok
    returns NodeStatus.FAIL if charge below limits
    """

    def __init__(self, name, topic_name, queue_size=1, *args, **kwargs):
        super(ChargeOKMonitor, self).__init__(
            name=name, topic_name=topic_name, topic_type=BatteryState,
            cb=self.charge_ok_cb, queue_size=1, *args, **kwargs)

    def charge_ok_cb(self, msg, nodedata):
        if msg.percentage > nodedata.get_data('min_charge', 1.):
            return NodeStatus(NodeStatus.SUCCESS, "charge above min")
        return NodeStatus(NodeStatus.FAIL, "charge below min")
