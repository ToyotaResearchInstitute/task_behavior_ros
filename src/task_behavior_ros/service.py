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

from task_behavior_engine.tree import Node
from task_behavior_engine.tree import NodeStatus


class ServiceClient(Node):

    """ Helper node to call a service
        To use:
            Instantiate this class, pass in service name, service type and either
            requeset or request callback function

        @param name [str] The name of this node
        @param service_name [str] The name of the service
        @param service_type [type] The type for the service
        @param request [msg] The request to send the action server on configure
        @param request_cb [function] The request callback function to call if no request is set
        @param server_timeout [float] The amount of time to wait for the server to be connected

        request_cb prototype:
            @param nodedata [NodeData] The nodedata for this node
            @returns [tuple] The request to send the action server on configure
    """

    def __init__(self, name, service_name, service_type, request=None,
                 request_cb=None, server_timeout=0.5, *args, **argv):
        super(ServiceClient, self).__init__(name=name, configure_cb=self.config,
                                            run_cb=self.run, *args, **argv)
        rospy.loginfo("[" + self._name + "] Creating ServiceClient")
        self.service_name = service_name
        self.service_type = service_type
        self.request = request
        self.request_cb = request_cb
        self.server_timeout = server_timeout
        self.server_connected = False

    def config(self, nodedata):
        rospy.loginfo(
            "[" + self._name + "] Waiting for server: " + self.service_name)
        try:
            rospy.wait_for_service(
                service=self.service_name, timeout=self.server_timeout)
            self.server_connected = True
        except:
            rospy.logerr(
                "[" + self._name + "] Could not connect to server: " + self.service_name)
            self.server_connected = False

    def run(self, nodedata):
        if self.server_connected:
            try:
                client = rospy.ServiceProxy(
                    self.service_name, self.service_type)
                if self.request:
                    rospy.loginfo("Sending request to service")
                    nodedata.result = client(*self.request)
                    return NodeStatus(NodeStatus.SUCCESS)
                if self.request_cb:
                    rospy.loginfo(
                        "Sending service request callback to service")
                    nodedata.result = client(*self.request_cb(nodedata))
                    return NodeStatus(NodeStatus.SUCCESS)

                rospy.logerr("[" + self._name + "] No server request set")
                return NodeStatus(NodeStatus.FAIL, "No server request set")
            except:
                return NodeStatus(NodeStatus.FAIL, "Failed calling service")
        return NodeStatus(NodeStatus.FAIL, "Not connected to server")
