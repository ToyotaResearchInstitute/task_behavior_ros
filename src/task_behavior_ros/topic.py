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

from std_msgs.msg import String
from task_behavior_engine.tree import Behavior
from task_behavior_engine.tree import Node
from task_behavior_engine.tree import NodeStatus


class TopicTrigger(Behavior):

    """ Behavior to begin child indicated by message.
        This behavior returns the result of the triggered child,
        otherwise if no child is triggered this behavior fails

        @param name [str] The name of this behavior
        @param topic_name [str] The name of the trigger topic
    """

    def __init__(self, name, topic_name, *args, **kwargs):
        super(TopicTrigger, self).__init__(name,
                                           run_cb=self.run,
                                           configure_cb=self.configure,
                                           *args, **kwargs)
        self.trigger_sub = rospy.Subscriber(topic_name,
                                            String,
                                            self._trigger_cb,
                                            queue_size=1)
        self.child_map = {}
        self.child = None
        self.is_running = False

    def _trigger_cb(self, msg):
        rospy.logdebug("got trigger msg " + msg.data)
        if msg.data in self.child_map:
            rospy.loginfo("triggering " + msg.data)
            child = self.child_map[msg.data]
            if self.child and not self.child == child:
                rospy.loginfo("canceling " + self.child._name)
                self.child.cancel(child)
            self.child = child

    def configure(self, nodedata):
        for child in self._children:
            self.child_map[child._name] = child

    def run(self, nodedata):
        result = NodeStatus(NodeStatus.FAIL, str("Waiting for trigger.."))
        if not self.child == None:
            child_result = self.tick_child(self.child)
            if child_result.status == NodeStatus.CANCEL:
                self.child = None
            elif child_result.status == NodeStatus.FAIL:
                self.child = None
                result = child_result
            elif child_result.status == NodeStatus.SUCCESS:
                self.child = None
                result = child_result
            else:
                result = NodeStatus(
                    NodeStatus.ACTIVE, "Running "+self.child._name)

        for child in self._children:
            if child.get_status() == NodeStatus.ACTIVE:
                if self.child == None:
                    self.child = child
                    result = child.get_status()
                elif self.child is not child:
                    self.cancel_child(child)
        return result


class TopicMonitor(Node):

    """ Helper node to monitor a topic
        To use:
            Instantiate this class, pass in topic name, topic type, and callback

        @param name [str] The name of this node
        @param topic_name [str] The name of the topic to monitor
        @param topic_type [type] The type of the topic
        @param cb [function] The function to call when message received on topic
        @param queue_size [int] The size of the message queue
        @param latch [bool] Whether to latch the topic callback.  If true, this
                        will return the result from the last cb and not wait
                        for a new message to be received

        callback:
            @param msg: The message from the subscriber
            @param nodedata: The nodedata for this node
            @returns NodeStatus
    """

    def __init__(self, name, topic_name, topic_type, cb, queue_size=1, latch=False, *args, **kwargs):
        super(TopicMonitor, self).__init__(name=name,
                                           run_cb=self.run,
                                           configure_cb=self.config,
                                           cleanup_cb=self.cleanup,
                                           *args, **kwargs)
        self.msg = None
        self.cb = cb
        self.result = NodeStatus(NodeStatus.ACTIVE)
        self.is_running = False
        self.latch = latch
        self.topic_sub = rospy.Subscriber(
            topic_name, topic_type, self._monitor, queue_size=queue_size)

    def _monitor(self, msg):
        if self.is_running or self.latch:
            nodedata = self._blackboard.get_memory(self._id)
            self.result = self.cb(msg, nodedata)
            if not type(self.result) == NodeStatus:
                raise TypeError(
                    "TopicMonitor callback must return a NodeStatus, "
                    "instead returned " + str(type(self.result)))

    def config(self, nodedata):
        if not self.latch:
            self.result = NodeStatus(NodeStatus.ACTIVE)
        self.is_running = True

    def run(self, nodedata):
        rospy.loginfo("topic monitor: " + str(self.result))
        return self.result

    def cleanup(self, nodedata):
        self.is_running = False


class TopicPublisher(Node):

    """ Helper node to publish to a topic
        To use:
            Instantiate this class, pass in topic name, topic type and either
            static message or message callback
        @param name [str] The name of this node
        @param topic_name [str] The name of the topic to publish
        @param topic_type [type] The type of the topic to publish
        @param msg [*] The message to publish
        @param msg_cb [function] The function to call to create the message
        @param queue_size [int] The queue size for this topic
        @param latch [bool] Whether to latch the topic

        callback:
            @param nodedata: The nodedata for this
            @returns topic_type: The message to send
    """

    def __init__(self, name, topic_name, topic_type, msg=None, msg_cb=None, queue_size=1, latch=False, *args, **kwargs):
        super(TopicPublisher, self).__init__(
            name=name, run_cb=self.run, *args, **kwargs)
        self.msg = msg
        self.msg_cb = msg_cb
        self.topic_type = topic_type
        self.topic_pub = rospy.Publisher(
            topic_name, topic_type, queue_size=queue_size, latch=latch)

    def run(self, nodedata):
        rospy.loginfo("Publishing message")
        msg = None
        if self.msg:
            msg = self.msg
        elif self.msg_cb:
            msg = self.msg_cb(nodedata=nodedata)
        if msg:
            rospy.loginfo(msg)
            if not type(msg) == self.topic_type:
                raise TypeError("Topic publisher message must match publisher")
            self.topic_pub.publish(msg)
            return NodeStatus(NodeStatus.SUCCESS)
        else:
            return NodeStatus(NodeStatus.FAIL, "No message set")
