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

from nose.tools import assert_equal

import rospy
from sensor_msgs.msg import BatteryState

from task_behavior_engine.tree import NodeStatus
from task_behavior_engine.tree import Blackboard
from task_behavior_ros import batterystate


class TestChargeCompleteMonitor(object):

    def setUp(self):

        self.blackboard = Blackboard()
        self.charge_monitor = batterystate.ChargeCompleteMonitor(
            name="monitor", topic_name="/battery", latch=True, blackboard=self.blackboard)

        self.msg_pub = rospy.Publisher("/battery", BatteryState,
                                       queue_size=1)
        rospy.sleep(0.5)

    def test_cb(self):
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        # empty message should fail
        msg = BatteryState()
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # nominal message without max set (should default to 1).
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        msg.percentage = 0.99
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        # set max
        self.blackboard.save("max_charge", 0.95, self.charge_monitor._id)
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # discharging at max
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)


class TestChargeOKMonitor(object):

    def setUp(self):

        self.blackboard = Blackboard()
        self.charge_monitor = batterystate.ChargeOKMonitor(
            name="monitor", topic_name="/battery", latch=True, blackboard=self.blackboard)

        self.msg_pub = rospy.Publisher("/battery", BatteryState,
                                       queue_size=1)
        rospy.sleep(0.5)

    def test_cb(self):
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        # empty message should fail
        msg = BatteryState()
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # nominal message without min set (should default to 1).
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        msg.percentage = 0.3
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # set max
        self.blackboard.save("min_charge", 0.25, self.charge_monitor._id)
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # charging
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)
