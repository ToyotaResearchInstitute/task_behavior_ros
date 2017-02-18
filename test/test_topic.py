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
from nose.tools import assert_raises

from std_msgs.msg import String

from task_behavior_ros import topic
from task_behavior_engine.tree import NodeStatus
from task_behavior_engine import node
from task_behavior_engine.tree import Blackboard


class TestTopicTrigger(object):

    def setUp(self):
        # test to ensure named child is triggered
        self.topic_trig = topic.TopicTrigger(name="trigger",
                                             topic_name="/topic_trigger")
        self.continue_node = node.Continue(name="continue")
        self.success_node = node.Success(name="success")
        self.fail_node = node.Fail(name="fail")

        self.topic_trig.add_child(self.continue_node)
        self.topic_trig.add_child(self.success_node)
        self.topic_trig.add_child(self.fail_node)

        self.msg_pub = rospy.Publisher("/topic_trigger", String,
                                       queue_size=1, latch=True)

    def test_trigger_cb(self):
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        msg = String()
        msg.data = "continue"

        self.msg_pub.publish(msg)
        # give time for pub/sub to connect
        rospy.sleep(0.5)

        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(self.continue_node.get_status(), NodeStatus.ACTIVE)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        # triggering another node while one is active
        # should cancel active node and start new one
        msg.data = "success"
        self.msg_pub.publish(msg)
        # give time for pub/sub to connect
        rospy.sleep(0.5)
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal(self.continue_node.get_status().status, NodeStatus.CANCEL)
        assert_equal(self.success_node.get_status().status, NodeStatus.SUCCESS)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        # triggering another node after all have finished
        # should just run that node
        msg.data = "fail"
        self.msg_pub.publish(msg)
        rospy.sleep(0.5)
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status().status, NodeStatus.CANCEL)
        assert_equal(self.success_node.get_status().status, NodeStatus.SUCCESS)
        assert_equal(self.fail_node.get_status().status, NodeStatus.FAIL)

    def test_child_running(self):
        # For ease of use reasons, this state gets updated if a child is
        # forced to run
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.continue_node.tick()
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(self.continue_node.get_status(), NodeStatus.ACTIVE)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

    def test_force(self):
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.topic_trig.force(NodeStatus.SUCCESS)
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.topic_trig.force(NodeStatus.FAIL)
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.topic_trig.force(NodeStatus.ACTIVE)
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

    def test_force_child(self):
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.continue_node.tick()
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(self.continue_node.get_status(), NodeStatus.ACTIVE)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.continue_node.force(NodeStatus.SUCCESS)
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal(self.continue_node.get_status(), NodeStatus.SUCCESS)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

    def test_cancel(self):
        # test that canceling parent cancels
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.topic_trig.cancel()
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.CANCEL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        # test that canceling parent cancels children
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.continue_node.tick()
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(self.continue_node.get_status(), NodeStatus.ACTIVE)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.topic_trig.cancel()
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.CANCEL)
        assert_equal(self.continue_node.get_status(), NodeStatus.CANCEL)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

    def test_cancel_child(self):
        # test that canceling child sets cancel state
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.PENDING)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.continue_node.tick()
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(self.continue_node.get_status(), NodeStatus.ACTIVE)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)

        self.continue_node.cancel()
        result = self.topic_trig.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(self.continue_node.get_status(), NodeStatus.CANCEL)
        assert_equal(self.success_node.get_status().status, NodeStatus.PENDING)
        assert_equal(self.fail_node.get_status().status, NodeStatus.PENDING)


class TestTopicMonitor(object):

    def setUp(self):
        # test to ensure named child is triggered
        def callback(msg, nodedata):
            nodedata.count = nodedata.count + 1
            if nodedata.count > 1:
                return NodeStatus(NodeStatus.SUCCESS)
            return NodeStatus(NodeStatus.ACTIVE)

        self.blackboard = Blackboard()
        self.topic_monitor = topic.TopicMonitor(name="monitor",
                                                topic_name="/topic_monitor",
                                                topic_type=String,
                                                cb=callback,
                                                blackboard=self.blackboard)

        self.blackboard.save("count", 0, self.topic_monitor._id)
        self.blackboard.save("called", False, self.topic_monitor._id)
        self.msg_pub = rospy.Publisher("/topic_monitor", String,
                                       queue_size=1, latch=True)

    def test_cb(self):
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        msg = String()
        msg.data = "test_string"
        self.msg_pub.publish(msg)
        rospy.sleep(0.5)
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(1, self.blackboard.get('count', self.topic_monitor._id))

        self.msg_pub.publish(msg)
        rospy.sleep(0.5)
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal(2, self.blackboard.get('count', self.topic_monitor._id))

    def test_force(self):
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        self.topic_monitor.force(NodeStatus.FAIL)
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)

        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        msg = String()
        msg.data = "test_string"
        self.msg_pub.publish(msg)
        rospy.sleep(0.5)
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(1, self.blackboard.get('count', self.topic_monitor._id))

        self.topic_monitor.force(NodeStatus.FAIL)
        self.msg_pub.publish(msg)
        rospy.sleep(0.5)
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal(2, self.blackboard.get('count', self.topic_monitor._id))

    def test_cancel(self):
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        self.topic_monitor.cancel()
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.CANCEL)

        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)

        msg = String()
        msg.data = "test_string"
        self.msg_pub.publish(msg)
        rospy.sleep(0.5)
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.ACTIVE)
        assert_equal(1, self.blackboard.get('count', self.topic_monitor._id))

        self.topic_monitor.cancel()
        self.msg_pub.publish(msg)
        rospy.sleep(0.5)
        result = self.topic_monitor.tick()
        assert_equal(result.status, NodeStatus.CANCEL)
        assert_equal(2, self.blackboard.get('count', self.topic_monitor._id))


class TestTopicPublisher(object):

    def setUp(self):
        # test to ensure named child is triggered
        def callback(nodedata):
            nodedata.count = nodedata.count + 1
            msg = String()
            msg.data = str(nodedata.count)
            return msg

        msg = String()
        msg.data = "test_msg"
        self.blackboard = Blackboard()
        self.topic_msg = topic.TopicPublisher(name="publisher",
                                              topic_name="/topic_publisher",
                                              topic_type=String,
                                              msg=msg,
                                              latch=True)
        self.topic_msg_cb = topic.TopicPublisher(name="cb_publisher",
                                                 topic_name="/topic_cb",
                                                 topic_type=String,
                                                 msg_cb=callback,
                                                 latch=True,
                                                 blackboard=self.blackboard)

        self.blackboard.save("count", 0, self.topic_msg_cb._id)

    def test_msg(self):
        result = self.topic_msg.tick()
        msg = rospy.wait_for_message('/topic_publisher', String)
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal('test_msg', msg.data)

        self.topic_msg = topic.TopicPublisher(name="publisher",
                                              topic_name="/topic_publisher",
                                              topic_type=String,
                                              msg="msg",
                                              latch=True)

        assert_raises(Exception, self.topic_msg.tick)

    def test_msg_cb(self):
        result = self.topic_msg_cb.tick()
        msg = rospy.wait_for_message('/topic_cb', String)
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal('1', msg.data)

    def test_force(self):
        result = self.topic_msg_cb.tick()
        msg = rospy.wait_for_message('/topic_cb', String)
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal('1', msg.data)

        self.topic_msg_cb.force(NodeStatus.FAIL)
        result = self.topic_msg_cb.tick()
        msg = rospy.wait_for_message('/topic_cb', String)
        assert_equal(result.status, NodeStatus.FAIL)
        assert_equal('1', msg.data)

    def test_cancel(self):
        result = self.topic_msg_cb.tick()
        msg = rospy.wait_for_message('/topic_cb', String)
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal('1', msg.data)

        self.topic_msg_cb.cancel()
        result = self.topic_msg_cb.tick()
        msg = rospy.wait_for_message('/topic_cb', String)
        assert_equal(result.status, NodeStatus.CANCEL)
        assert_equal('1', msg.data)

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
