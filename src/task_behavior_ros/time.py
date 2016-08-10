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

from task_behavior_engine.tree import Decorator
from task_behavior_engine.tree import Node
from task_behavior_engine.tree import NodeStatus


class Timeout(Decorator):

    """ A timeout decorator returns FAIL if child timed out
        If the child returns SUCCESS, this will return SUCCESS
        If the child returns FAIL, this will return FAIL.
        If the child returns ACTIVE and timeout hasn't occured, this will return ACTIVE
        If the child returns ACTIVE and timeout has occured, this will return FAIL
        All other statuses are passed through.
    """

    def __init__(self, name, timeout, *args, **kwargs):
        super(Timeout, self).__init__(
            name=name, configure_cb=self.configure, run_cb=self.run, *args, **kwargs)
        rospy.loginfo("setting up time")
        self.timeout = timeout
        self.timeout_time = rospy.Time(0)

    def configure(self, nodedata):
        rospy.logdebug("Timeout.configure()" + self._child._name)
        self.timeout = nodedata.get_data('timeout', self.timeout)
        self.timeout_time = rospy.Time.now() + rospy.Duration(self.timeout)

    def run(self, nodedata):
        rospy.logdebug("Timeout.run()" + self._child._name)
        result = self.tick_child()
        if result == NodeStatus.SUCCESS or result == NodeStatus.FAIL:
            return result
        time_remaining = self.timeout_time - rospy.Time.now()
        if time_remaining < rospy.Duration(0):
            return NodeStatus(NodeStatus.FAIL, "Time limit reached")
        rospy.loginfo("time_remaining: " + str(time_remaining.to_sec()))

        return result


class TimedWait(Node):

    """ A timeout wait returns SUCCEED after time has lasped.
        @param name [str] The name of this node
        @param timeout [float] Amount of time to wait

        configurable nodedata:
            timeout [float] Amount of time to wait
    """

    def __init__(self, name, timeout, *args, **kwargs):
        super(TimedWait, self).__init__(
            name=name, configure_cb=self.configure, run_cb=self.run, *args, **kwargs)
        rospy.loginfo("setting up time")
        self.timeout = timeout
        self.timeout_time = rospy.Time(0)

    def configure(self, nodedata):
        rospy.logdebug("TimedWait.configure()" + self._name)
        self.timeout = nodedata.get_data('timeout', self.timeout)
        self.timeout_time = rospy.Time.now() + rospy.Duration(self.timeout)

    def run(self, nodedata):
        rospy.logdebug("TimedWait.run()" + self._name)
        time_remaining = self.timeout_time - rospy.Time.now()
        if time_remaining < rospy.Duration(0):
            return NodeStatus(NodeStatus.SUCCESS, "Wait time finished")
        rospy.loginfo("wait time_remaining: " + str(time_remaining.to_sec()))

        return NodeStatus(NodeStatus.ACTIVE)
