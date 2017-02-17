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

    def test_init(self):
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        # empty message should fail
        msg = BatteryState()
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_charging(self):
        msg = BatteryState()
        # Charging at max, report ACTIVE
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        msg.percentage = 1.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        # Charging but above max, report SUCCESS
        self.blackboard.save("max_charge", 0.95, self.charge_monitor._id)
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Charging but below max, report ACTIVE
        msg.percentage = 0.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)  # let callbacks finish
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

    def test_discharging(self):
        msg = BatteryState()
        # Discharging at max, report FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.percentage = 1.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Discharging but above max, report FAIL
        self.blackboard.save("max_charge", 0.95, self.charge_monitor._id)
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Discharging but below max, report FAIL
        msg.percentage = 0.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)  # let callbacks finish
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_full(self):
        msg = BatteryState()
        # Charge at full, report SUCCESS.
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        msg.percentage = 1.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Charge at full but above max, report SUCCESS
        self.blackboard.save("max_charge", 0.95, self.charge_monitor._id)
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Charge at full, but percentage low, still report success
        msg.percentage = 0.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)  # let callbacks finish
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

    def test_unknown(self):
        msg = BatteryState()
        # Unkown at max, report FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.percentage = 1.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Unkown but above max, report FAIL
        self.blackboard.save("max_charge", 0.95, self.charge_monitor._id)
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Unknown but below max, report FAIL
        msg.percentage = 0.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)  # let callbacks finish
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

    def test_init(self):
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        # empty message should fail
        msg = BatteryState()
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_charging(self):
        msg = BatteryState()
        # Charging at min, report FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        msg.percentage = 0.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Charging but above min, report SUCCESS
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.8
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Charging but below min, report FAIL
        msg.percentage = .1
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)  # let callbacks finish
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_discharging(self):
        msg = BatteryState()
        # Discharging at max, report FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.percentage = 0.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Discharging but above min, report SUCCESS
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.8
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Discharging but below min, report FAIL
        msg.percentage = .1
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)  # let callbacks finish
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_full(self):
        msg = BatteryState()
        # Charge at full, report FAIL.
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        msg.percentage = 0.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Charge at full but above min, report SUCCESS
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.8
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Charge at full but below min, report FAIL
        msg.percentage = .1
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)  # let callbacks finish
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_unknown(self):
        msg = BatteryState()
        # Unkown at max, report FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.percentage = 0.
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Unknown but above min, report SUCCESS
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.8
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Unknown but below min, report FAIL
        msg.percentage = .1
        self.msg_pub.publish(msg)
        rospy.sleep(0.1)  # let callbacks finish
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)
