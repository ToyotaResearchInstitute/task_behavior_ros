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

import sys

import rospy
import tf2_ros
import tf2_kdl

from task_behavior_engine.tree import Node
from task_behavior_engine.tree import NodeStatus


class GetTransform(Node):

    """ This node will return NodeStatus.SUCCESS if the TF can be found
        otherwise it will return NodeStatus.FAIL
        @param name [str] The name of this node
        @param frame_id [str] The name of the TF to get
        @param ref_frame [str] the reference frame for the TF
        @param buffer [float] The amount of time to wait for a transform
        @param bounds [array] The bounds for a valid transform, expressed in ref_frame
                              format: [(x_min, x_max), (y_min, y_max), (z_min, z_max)]
    """

    def __init__(self, name, frame_id, ref_frame="map", buffer=5.0, bounds=None, *args, **kwargs):
        super(GetTransform, self).__init__(name=name,
                                           configure_cb=self.configure,
                                           run_cb=self.run,
                                           cleanup_cb=self.cleanup,
                                           *args, **kwargs)
        self.frame_id = frame_id
        self.ref_frame = ref_frame
        self.bounds = bounds
        self.buffer = buffer
        self.tfBuffer = tf2_ros.BufferClient('tf2_buffer_server')

    def configure(self, nodedata):
        self.frame_id = nodedata.get_data('frame_id', self.frame_id)
        self.ref_frame = nodedata.get_data('ref_frame', self.ref_frame)
        self.bounds = nodedata.get_data('bounds', self.bounds)

    def run(self, nodedata):
        rospy.loginfo(
            "Looking for transform " + self.frame_id + "->"+self.ref_frame)
        try:
            nodedata.transform = self.tfBuffer.lookup_transform(
                self.ref_frame,
                self.frame_id,
                rospy.Time.now(),
                rospy.Duration(self.buffer))
            rospy.loginfo("found transform")
            rospy.loginfo(nodedata.transform)
            if not self.bounds == None:
                [(x_min, x_max), (y_min, y_max), (z_min, z_max)] = self.bounds
                if not (nodedata.transform.transform.translation.x > x_min and
                        nodedata.transform.transform.translation.x < x_max and
                        nodedata.transform.transform.translation.y > y_min and
                        nodedata.transform.transform.translation.y < y_max and
                        nodedata.transform.transform.translation.z > z_min and
                        nodedata.transform.transform.translation.z < z_max):
                    rospy.logerr("Transform not within bounds")
                    return NodeStatus(NodeStatus.FAIL, "Transform not within bounds")
        except:
            rospy.logerr("Could not find transform")
            e = sys.exc_info()
            rospy.logerr(e)
            return NodeStatus(NodeStatus.FAIL, "Could not find transform: " + str(e))
        rospy.loginfo("Found transform")
        return NodeStatus(NodeStatus.SUCCESS, "Found transform")

    def cleanup(self, nodedata):
        pass


class TransformMonitor(Node):

    """ This node will monitor a transform.  If the transform changes
        by more than delta, then this node returns NodeStatus.FAIL,
        otherwise it returns NodeStatus.SUCCESS.
        @param frame_id [str] The name of the frame to monitor
        @param ref_frame [str] The name reference frame for the transform
        @param delta_lin [float] The amount of acceptable linear motion (in m)
        @param delta_ang [float] The amount of acceptable angular motion (in rad)
    """

    def __init__(self, name, frame_id, ref_frame="map",
                 delta_lin=0.2, delta_ang=0.5, timeout=None, *args, **kwargs):
        super(TransformMonitor, self).__init__(name=name,
                                               configure_cb=self.configure,
                                               run_cb=self.run,
                                               cleanup_cb=self.cleanup,
                                               *args, **kwargs)
        self.frame_id = frame_id
        self.ref_frame = ref_frame
        self.delta_lin = delta_lin
        self.delta_ang = delta_ang
        self.timeout = timeout
        self.time = rospy.Time(0)
        self.tfBuffer = tf2_ros.BufferClient('tf2_buffer_server')

    def configure(self, nodedata):
        self.frame_id = nodedata.get_data('frame_id',  self.frame_id)
        self.ref_frame = nodedata.get_data('ref_frame', self.ref_frame)
        self.delta_lin = nodedata.get_data('delta_lin', self.delta_lin)
        self.delta_ang = nodedata.get_data('delta_ang', self.delta_ang)
        self.timeout = nodedata.get_data('timeout',   self.timeout)
        self.transform = nodedata.get_data('transform', None)

    def run(self, nodedata):
        rospy.loginfo(
            "Tracking transform " + self.frame_id + "->" + self.ref_frame)
        try:
            curr_transform = self.tfBuffer.lookup_transform(self.frame_id,
                                                            self.ref_frame,
                                                            rospy.Time())
            if self.transform == None:
                self.transform = curr_transform

            f_curr = tf2_kdl.transform_to_kdl(curr_transform)
            f_prev = tf2_kdl.transform_to_kdl(self.transform)

            twist = tf2_kdl.PyKDL.diff(f_curr, f_prev)
            rospy.loginfo("Frame diff:" + str(twist))
            for i in range(0, 3):
                if (abs(twist[i]) > self.delta_lin):
                    rospy.logerr("position moved too much")
                    return NodeStatus(NodeStatus.FAIL, "position moved too much")

            for i in range(3, 6):
                if (abs(twist[i]) > self.delta_ang):
                    rospy.logerr("angle moved too much")
                    return NodeStatus(NodeStatus.FAIL, "angle moved too much")

            self.time = rospy.Time.now()
            return NodeStatus(NodeStatus.ACTIVE, "Transform ok")
        except:
            rospy.logerr("Could not find transform")
            if self.timeout and rospy.Time.now() - self.time > rospy.Duration(self.timeout):
                return NodeStatus(NodeStatus.FAIL, "Transform not found after timeout")
            return NodeStatus(NodeStatus.ACTIVE, "Waiting for transform")

    def cleanup(self, nodedata):
        self.transform = None
