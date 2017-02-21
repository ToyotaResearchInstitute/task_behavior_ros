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

import nose
from nose.tools import assert_equal

import argparse
import rospy
import sys

from task_behavior_engine import node
from task_behavior_engine.tree import NodeStatus
from task_behavior_ros import time


class TestTimeout(object):

    def test_success(self):
        success_node = node.Success(name="success")
        timeout_success = time.Timeout(name="timed_success",
                                       timeout=0.5,
                                       child=success_node)

        result = timeout_success.tick()
        assert_equal(result, NodeStatus.SUCCESS)

    def test_fail(self):
        fail_node = node.Fail(name="fail")
        timeout_fail = time.Timeout(name="timed_fail",
                                    timeout=0.5,
                                    child=fail_node)

        result = timeout_fail.tick()
        assert_equal(result, NodeStatus.FAIL)

    def test_timedout(self):
        active_node = node.Continue(name="continue")
        timeout_active = time.Timeout(name="timed_continue",
                                      timeout=0.5,
                                      child=active_node)

        result = timeout_active.tick()
        assert_equal(result, NodeStatus.ACTIVE)
        rospy.sleep(0.5)

        result = timeout_active.tick()
        assert_equal(result, NodeStatus.FAIL)
        assert_equal(active_node.get_status(), NodeStatus.CANCEL)

    def test_force(self):
        active_node = node.Continue(name="continue")
        timeout_active = time.Timeout(name="timed_continue",
                                      timeout=0.5,
                                      child=active_node)

        result = timeout_active.tick()
        assert_equal(result, NodeStatus.ACTIVE)
        timeout_active.force(NodeStatus.SUCCESS)

        rospy.sleep(0.5)

        result = timeout_active.tick()
        assert_equal(result, NodeStatus.SUCCESS)
        assert_equal(active_node.get_status(), NodeStatus.CANCEL)

    def test_cancel(self):
        active_node = node.Continue(name="continue")
        timeout_active = time.Timeout(name="timed_continue",
                                      timeout=0.5,
                                      child=active_node)

        result = timeout_active.tick()
        assert_equal(result, NodeStatus.ACTIVE)
        timeout_active.cancel()

        end = rospy.Time.now() + rospy.Duration(0.5)
        while(rospy.Time.now() < end):
            rospy.sleep(.1)

        result = timeout_active.tick()
        assert_equal(result, NodeStatus.CANCEL)
        assert_equal(active_node.get_status(), NodeStatus.CANCEL)


class TestTimedWait(object):

    def test_time(self):
        timed_node = time.TimedWait(name="timer",
                                    timeout=0.5)

        result = timed_node.tick()
        assert_equal(result, NodeStatus.ACTIVE)

        rospy.sleep(0.5)

        result = timed_node.tick()
        assert_equal(result, NodeStatus.SUCCESS)

    def test_force(self):
        timed_node = time.TimedWait(name="timer",
                                    timeout=0.5)

        result = timed_node.tick()
        assert_equal(result, NodeStatus.ACTIVE)

        timed_node.force(NodeStatus.FAIL)
        result = timed_node.tick()
        assert_equal(result, NodeStatus.FAIL)

        result = timed_node.tick()
        assert_equal(result, NodeStatus.ACTIVE)
        timed_node.force(NodeStatus.FAIL)
        rospy.sleep(0.5)
        result = timed_node.tick()
        assert_equal(result, NodeStatus.FAIL)

    def test_cancel(self):
        timed_node = time.TimedWait(name="timer",
                                    timeout=0.5)

        result = timed_node.tick()
        assert_equal(result, NodeStatus.ACTIVE)

        timed_node.cancel()
        result = timed_node.tick()
        assert_equal(result, NodeStatus.CANCEL)

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
