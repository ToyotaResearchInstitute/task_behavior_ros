#!/usr/bin/env python
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
import argparse
import rospy
import sys

import nose
from nose.tools import assert_equal

from sensor_msgs.msg import BatteryState

from task_behavior_engine.tree import NodeStatus
from task_behavior_engine.tree import Blackboard
from task_behavior_ros import batterystate


class TestChargeCompleteMonitor(object):

    def setUp(self):
        self.blackboard = Blackboard()
        self.charge_monitor = batterystate.ChargeCompleteMonitor(
            name="monitor",
            topic_name="battery",
            latch=True,
            blackboard=self.blackboard)

    def test_init(self):
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        # empty message should fail
        msg = BatteryState()
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_charging(self):
        msg = BatteryState()

        # Charging no max set, FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Charging at max, report SUCCESS
        msg.percentage = 0.9
        self.blackboard.save("max_charge", 0.9, self.charge_monitor._id)
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Charging but above max, report SUCCESS
        msg.percentage = 1.0
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Charging but below max, report ACTIVE
        msg.percentage = 0.
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

    def test_discharging(self):
        msg = BatteryState()

        # Discharging no max set, FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Discharging at max, report FAIL
        msg.percentage = 0.95
        self.blackboard.save("max_charge", 0.95, self.charge_monitor._id)
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Discharging but above max, report FAIL
        msg.percentage = 1.0
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Discharging but below max, report FAIL
        msg.percentage = 0.
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_full(self):
        msg = BatteryState()

        # Full but no max set, report SUCCESS
        # If the battery is reporting full, regardless of charge percentage
        # report SUCCESS
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Full but at max, report SUCCESS.
        msg.percentage = 0.95
        self.blackboard.save("max_charge", 0.95, self.charge_monitor._id)
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Full but above max, report SUCCESS
        msg.percentage = 1.0
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Full but below max, still report success
        msg.percentage = 0.
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

    def test_unknown(self):
        msg = BatteryState()

        # Unknown but no max set, FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Unkown at max, report FAIL
        self.blackboard.save("max_charge", 0.95, self.charge_monitor._id)
        msg.percentage = 0.95
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Unkown but above max, report FAIL
        msg.percentage = 1.0
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Unknown but below max, report FAIL
        msg.percentage = 0.
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)


class TestChargeOKMonitor(object):

    def setUp(self):
        self.blackboard = Blackboard()
        self.charge_monitor = batterystate.ChargeOKMonitor(
            name="monitor", topic_name="battery", latch=True, blackboard=self.blackboard)

    def test_init(self):
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        # empty message should fail
        msg = BatteryState()
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_charging(self):
        msg = BatteryState()

        # Charging but no min set, FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Charging at min, report FAIL
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.5
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Charging but above min, report SUCCESS
        msg.percentage = 0.8
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Charging but below min, report FAIL
        msg.percentage = .1
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_discharging(self):
        msg = BatteryState()

        # Dischaging but no min set, FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Discharging at min, report FAIL
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.5
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Discharging but above min, report SUCCESS
        msg.percentage = 0.8
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Discharging but below min, report FAIL
        msg.percentage = .1
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_full(self):
        msg = BatteryState()

        # Full but no min set, FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Full but at min, report FAIL.
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.5
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Full but above min, report SUCCESS
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.8
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Full but below min, report FAIL
        msg.percentage = .1
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_unknown(self):
        msg = BatteryState()

        # Unknown but no min set, FAIL
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Unknown but at min, report FAIL
        self.blackboard.save("min_charge", 0.5, self.charge_monitor._id)
        msg.percentage = 0.5
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        # Unknown but above min, report SUCCESS
        msg.percentage = 0.8
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)

        # Unknown but below min, report FAIL
        msg.percentage = .1
        self.charge_monitor.topic_sub.callback(msg)
        result = self.charge_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

if __name__ == '__main__':
    # This code will run the test in this file.'
    module_name = sys.modules[__name__].__file__

    parser = argparse.ArgumentParser(description='Perform unit test.')
    parser.add_argument(
        '--gtest_output', nargs='?', default='test.xml')

    args, unknown = parser.parse_known_args()

    noseargs = [sys.argv[0], module_name, '--with-xunit',
                '--xunit-file='+str(args.gtest_output.lstrip('xml:'))]
    nose.run(argv=noseargs)
