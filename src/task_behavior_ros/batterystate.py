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
    @param topic_name [str] The name of the topic

    NodeData:
        @param max_charge [float] Number between (0., 1.) that indicates
            charging percentage that will qualify as complete

    returns NodeStatus.SUCCESS if charging complete
    returns NodeStatus.ACTIVE if charging
    returns NodeStatus.FAIL if not charging
    """

    def __init__(self, name, topic_name, *args, **kwargs):
        super(ChargeCompleteMonitor, self).__init__(
            name=name, topic_name=topic_name, topic_type=BatteryState,
            cb=self.charge_complete_cb, *args, **kwargs)

    def charge_complete_cb(self, msg, nodedata):
        rospy.loginfo('received msg: ' + str(msg))
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_FULL:
            return NodeStatus(NodeStatus.SUCCESS, "Power supply state full")

        max_charge = nodedata.get_data('max_charge', None)
        if not max_charge:
            return NodeStatus(NodeStatus.FAIL, "max_charge is not set")

        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            if round(msg.percentage, 5) >= max_charge:
                return NodeStatus(NodeStatus.SUCCESS, "charge at or above max")
            else:
                return NodeStatus(NodeStatus.ACTIVE, "charging")
        return NodeStatus(NodeStatus.FAIL, "not charging")


class ChargeOKMonitor(TopicMonitor):

    """
    Checks if msg indicates charge is ok.
    @param name [str] The name of this node
    @param topic_name [str] The name of the topic

    NodeData:
       @param min_charge [float] Number between (0., 1.) that indicates minimum charging percentage

    returns NodeStatus.SUCCESS if charge above min
    returns NodeStatus.FAIL if charge at or below min
    """

    def __init__(self, name, topic_name, *args, **kwargs):
        super(ChargeOKMonitor, self).__init__(
            name=name, topic_name=topic_name, topic_type=BatteryState,
            cb=self.charge_ok_cb, *args, **kwargs)

    def charge_ok_cb(self, msg, nodedata):
        min_charge = nodedata.get_data('min_charge', None)
        if not min_charge:
            return NodeStatus(NodeStatus.FAIL, "min_charge is not set")

        if msg.percentage > min_charge:
            return NodeStatus(NodeStatus.SUCCESS, "charge above min")
        return NodeStatus(NodeStatus.FAIL, "charge below min")
