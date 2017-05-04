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

import actionlib
from actionlib_msgs.msg import GoalStatus

from task_behavior_engine.tree import Node
from task_behavior_engine.tree import NodeStatus


class ActionClient(Node):

    """ Helper node to call an action server
        To use:
            Instantiate this class, pass in action name, action type and either
            goal or goal callback function

        @param name [str] The name of this node
        @param action_name [str] The name of the action server
        @param action_type [type] The type for the action
        @param goal [msg] The goal to send the action server on configure
        @param goal_cb [function] The goal callback function to call if no goal is set
        @param server_timeout [float] The amount of time to wait for the server to be connected

        goal_cb prototype:
            @param nodedata [NodeData] The nodedata for this node
            @returns [msg] The goal to send the action server on configure
    """

    def __init__(self, name, action_name, action_type, goal=None, goal_cb=None,
                 server_timeout=0.5, *args, **kwargs):
        super(ActionClient, self).__init__(name=name,
                                           configure_cb=self.config,
                                           run_cb=self.run,
                                           cleanup_cb=self.cleanup,
                                           *args, **kwargs)
        rospy.loginfo("[" + self._name + "] Creating client")
        self.client = actionlib.SimpleActionClient(action_name, action_type)
        self.goal = goal
        self.goal_cb = goal_cb
        self.goal_msg = None
        self.server_timeout = server_timeout
        self.server_connected = False

    def config(self, nodedata):
        rospy.loginfo(
            "[" + self._name + "] Waiting for server: " + self.client.action_client.ns)
        if self.client.wait_for_server(rospy.Duration(self.server_timeout)):
            self.server_connected = True
            rospy.loginfo("[" + self._name + "] Found server")
            self.goal_msg = None
            if self.goal:
                self.goal_msg = self.goal
            elif self.goal_cb:
                self.goal_msg = self.goal_cb(nodedata)
            if self.goal_msg:
                self.client.send_goal(self.goal_msg)
                rospy.loginfo("sent goal")
        else:
            self.server_connected = False

    def run(self, nodedata):
        if self.server_connected:
            if self.goal_msg:
                nodedata.state = self.client.get_state()
                if nodedata.state == GoalStatus.PENDING or nodedata.state == GoalStatus.ACTIVE:
                    rospy.loginfo("[" + self._name + "] state: active")
                    return NodeStatus(NodeStatus.ACTIVE,
                                      self.client.get_goal_status_text())
                elif nodedata.state == GoalStatus.SUCCEEDED:
                    nodedata.result = self.client.get_result()
                    rospy.loginfo("[" + self._name + "] state: succeeded")
                    return NodeStatus(NodeStatus.SUCCESS,
                                      self.client.get_goal_status_text())
                else:
                    rospy.loginfo("[" + self._name + "] state: failed")
                    return NodeStatus(NodeStatus.FAIL,
                                      self.client.get_goal_status_text())
            else:
                rospy.logerr("[" + self._name + "] no goal set")
                return NodeStatus(NodeStatus.FAIL, "No goal set")
        rospy.logerr("[" + self._name + "] client not connected to server")
        return NodeStatus(NodeStatus.FAIL, "Client not connected to server")

    def cleanup(self, nodedata):
        rospy.loginfo("[" + self._name + "] cleaning up...")
        if self.server_connected and self.client.get_state() <= GoalStatus.ACTIVE:
            rospy.loginfo("[" + self._name + "] canceling goal..")
            self.client.cancel_goal()
