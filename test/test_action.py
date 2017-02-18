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

import actionlib
from actionlib_msgs.msg import GoalStatus
import actionlib_tutorials.msg

from task_behavior_engine.tree import NodeData
from task_behavior_engine.tree import NodeStatus
from task_behavior_ros import action


class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                actionlib_tutorials.msg.FibonacciAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (
            self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

        # start executing the action
        for i in xrange(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(
                self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for
            # demonstration purposes
            r.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


def setup_module():
    FibonacciAction('test_action')


class TestActionClient(object):

    def test_server_connected(self):
        # test nominal condition
        ac = action.ActionClient(name="actionlib_tutorial_client",
                                 action_name='/test_action',
                                 action_type=actionlib_tutorials.msg.FibonacciAction)
        ac.config(NodeData())
        assert_equal(ac.server_connected, True)

        # test server not available
        ac = action.ActionClient(name="test_no_server",
                                 action_name="/no_server",
                                 action_type=actionlib_tutorials.msg.FibonacciAction)
        ac.config(NodeData())
        assert_equal(ac.server_connected, False)

    def test_goal(self):
        msg = actionlib_tutorials.msg.FibonacciGoal()
        msg.order = 3
        # test nominal condition
        ac = action.ActionClient(name="actionlib_tutorial_client",
                                 action_name='/test_action',
                                 action_type=actionlib_tutorials.msg.FibonacciAction,
                                 goal=msg)

        result = ac.tick()
        assert_equal(result, NodeStatus.ACTIVE)

        rospy.sleep(3.)

        result = ac.tick()
        assert_equal(result, NodeStatus.SUCCESS)

    def test_goal_cb(self):

        def goal_cb(nodedata):
            msg = actionlib_tutorials.msg.FibonacciGoal()
            msg.order = nodedata.get_data('order', 3)
            return msg

        ac = action.ActionClient(name="actionlib_tutorial_client",
                                 action_name="/test_action",
                                 action_type=actionlib_tutorials.msg.FibonacciAction,
                                 goal_cb=goal_cb)

        result = ac.tick()
        assert_equal(result, NodeStatus.ACTIVE)

        rospy.sleep(3.)

        result = ac.tick()
        assert_equal(result, NodeStatus.SUCCESS)

    def test_force(self):
        msg = actionlib_tutorials.msg.FibonacciGoal()
        msg.order = 3
        # test nominal condition
        ac = action.ActionClient(name="actionlib_tutorial_client",
                                 action_name='/test_action',
                                 action_type=actionlib_tutorials.msg.FibonacciAction,
                                 goal=msg)

        result = ac.tick()
        assert_equal(result, NodeStatus.ACTIVE)
        rospy.sleep(1.)

        ac.force(NodeStatus.FAIL)
        result = ac.tick()
        assert_equal(result, NodeStatus.FAIL)
        rospy.sleep(1.1)
        assert_equal(ac.client.get_state(), GoalStatus.PREEMPTED)

    def test_cancel(self):
        msg = actionlib_tutorials.msg.FibonacciGoal()
        msg.order = 3
        # test nominal condition
        ac = action.ActionClient(name="actionlib_tutorial_client",
                                 action_name='/test_action',
                                 action_type=actionlib_tutorials.msg.FibonacciAction,
                                 goal=msg)

        result = ac.tick()
        assert_equal(result, NodeStatus.ACTIVE)
        rospy.sleep(1.)

        ac.cancel()
        result = ac.tick()
        assert_equal(result, NodeStatus.CANCEL)
        rospy.sleep(0.9)
        assert_equal(ac.client.get_state(), GoalStatus.PREEMPTED)

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
