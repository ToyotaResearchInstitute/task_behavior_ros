# Copyright 2016 Toyota Research Instutute

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
from task_behavior_engine.tree import Behavior
from task_behavior_engine.tree import Decorator

from std_msgs.msg import Empty
from std_msgs.msg import String
from task_behavior_engine.tree import NodeStatus
from task_behavior_msgs.msg import NodeData
from task_behavior_msgs.msg import TreeDataDump
from task_behavior_msgs.msg import TreeNode
from task_behavior_msgs.msg import TreeNodeStatus
from task_behavior_msgs.msg import TreeStatus
from task_behavior_msgs.msg import TreeStructure


class Introspection(object):

    """ This class enables ros introspection on a behavior tree
    """

    def __init__(self, node, prefix="tree"):
        """ recursively set up the pub/subs of this node
            @param node [Node] The node to recursively introspect
            @param prefix [string] name prefix for the set of topics
        """
        self.parent = node
        self.node_map = {}

        self._structure_pub = rospy.Publisher("%s/structure" % prefix,
                                              TreeStructure,
                                              queue_size=1,
                                              latch=True)
        self._status_pub = rospy.Publisher("%s/status" % prefix,
                                           TreeStatus,
                                           queue_size=1,
                                           latch=True)
        self._data_dump = rospy.Publisher("%s/data" % prefix,
                                          TreeDataDump,
                                          queue_size=1,
                                          latch=True)

        self._reload()

        self._force_sub = rospy.Subscriber("%s/force" % prefix,
                                           TreeStatus,
                                           self._force_node,
                                           queue_size=1)
        self._tick_sub = rospy.Subscriber("%s/tick" % prefix,
                                          String,
                                          self._tick_node,
                                          queue_size=1)
        self._cancel_sub = rospy.Subscriber("%s/cancel" % prefix,
                                            String,
                                            self._cancel_node,
                                            queue_size=1)
        self._refresh_sub = rospy.Subscriber("%s/refresh" % prefix,
                                             Empty,
                                             self._reload,
                                             queue_size=1)
        self._clear_sub = rospy.Subscriber("%s/clear" % prefix,
                                           Empty,
                                           self._clear,
                                           queue_size=1)

    def _reload(self, msg=None):
        """ recreate the node map
            @param msg [Empty] Optional empty message for callback
        """
        self._create_node_map(self.parent)
        self.publish_structure()

    def _clear(self, msg=None):
        """ reset the node status state to PENDING
            @param msg [Empty] Optional empty message for callback
        """
        self._clear_nodes(self.parent)

    def _clear_nodes(self, node):
        """ recursive function to reset the node state to PENDING
            @param node [Node] The node to reset the state of
        """
        node._blackboard.clear_node_status()

        if Behavior in type(node).__bases__:
            for child in node._children:
                self._clear_nodes(child)
        elif Decorator in type(node).__bases__:
            self._clear_nodes(node._child)

    def _create_node_map(self, node):
        """  Recursive function to discover the parent/children nodes
            @param node [Node] The node to recursively add to the map
        """
        self.node_map[str(node._id)] = node
        if Behavior in type(node).__bases__:
            for child in node._children:
                self._create_node_map(child)
        if Decorator in type(node).__bases__:
            self._create_node_map(node._child)

    def publish_structure(self):
        """ Publish the behavior tree structure
        """
        tree_structure_msg = self._create_structure_msg(self.parent,
                                                        TreeStructure())
        tree_structure_msg.header.stamp = rospy.Time.now()
        self._structure_pub.publish(tree_structure_msg)

    def _create_structure_msg(self, node, msg=None):
        """ Recursively create the behavior tree structure message
            @param node [Node] The base node to create the structure from
            @param msg [TreeStructure] The message created so far
            @returns [TreeStructure] The structure message
        """
        if msg is None:
            msg = TreeStructure()
        node_msg = TreeNode()
        node_msg.id = str(node._id)
        node_msg.name = node._name

        if Behavior in type(node).__bases__:
            node_msg.type = TreeNode.BEHAVIOR
            for child in node._children:
                node_msg.children.append(str(child._id))
            msg.node.append(node_msg)
            for child in node._children:
                msg = self._create_structure_msg(child, msg)
        elif Decorator in type(node).__bases__:
            node_msg.type = TreeNode.DECORATOR
            node_msg.children = [str(node._child._id)]
            msg.node.append(node_msg)
            msg = self._create_structure_msg(node._child, msg)
        else:
            msg.node.append(node_msg)

        return msg

    def publish_status(self):
        """ Publish the status message for the behavior tree
        """
        tree_status_msg = self._create_status_msg(self.parent, TreeStatus())
        tree_status_msg.header.stamp = rospy.Time.now()
        self._status_pub.publish(tree_status_msg)

    def publish_data_dump(self, node=None):
        """
        Publishes a data dump message of the tree
        """
        if node is None:
            node = self.parent
        tree_data_dump_msg = self._create_tree_data_dump_msg(
            node, TreeDataDump())
        tree_data_dump_msg.header.stamp = rospy.Time.now()
        self._data_dump.publish(tree_data_dump_msg)

    def _create_node_data_msg(self, node):
        """ Create a node's data dump message
        @param node [Node] The node to get the data from
        @returns [NodeData] The blackboard data from the node
        """
        nodedata = node.get_nodedata()
        ndd = NodeData()
        for key in nodedata.keys():
            ndd.key.append(str(key))
            ndd.value.append(str(nodedata.get_data(key)))
        return ndd

    def _create_tree_data_dump_msg(self, node, msg=None):
        """ Create the status message
            @param node [Node] The node to get the status from (recursive)
            @param msg [TreeDataDump] The message created so far
            @returns [TreeDataDump] The status message
        """
        if msg is None:
            msg = TreeDataDump()
        msg.structure = self._create_structure_msg(node, TreeStructure())
        msg.status = self._create_status_msg(node, TreeStatus())

        return msg

    def _create_status_msg(self, node, msg=None):
        """ Create the status message
            @param node [Node] The node to get the status from (recursive)
            @param msg [TreeStatus] The current TreeStatus message
            @returns [TreeStatus] The status message
        """
        if msg is None:
            msg = TreeStatus()
        node_status = node.get_status()
        node_status_msg = TreeNodeStatus()
        node_status_msg.status = node_status.status
        node_status_msg.text = node_status.text

        msg.id.append(str(node._id))
        msg.name.append(node._name)
        msg.status.append(node_status_msg)
        nodedata = self._create_node_data_msg(node)
        msg.data.append(nodedata)

        if Behavior in type(node).__bases__:
            for child in node._children:
                msg = self._create_status_msg(child, msg)
        if Decorator in type(node).__bases__:
            msg = self._create_status_msg(node._child, msg)
        return msg

    def _force_node(self, msg):
        """ Force a node into a certain NodeStatus state
            @param msg [TreeStatus] The node status to force
        """
        if not len(msg.id) == len(msg.status):
            rospy.logerr("msg.id and msg.status have mismatched lengths")
            return
        for i, node_id in enumerate(msg.id):
            if node_id in self.node_map.keys():
                rospy.loginfo(
                    "forcing " + str(node_id) + " to " + str(msg.status[i].status))
                self.node_map[node_id].force(msg.status[i].status)

    def _tick_node(self, msg):
        """ Helper function to update a node
            @param msg [String] The id of the node to update
        """
        if msg.data in self.node_map:
            self.node_map[msg.data].tick()

    def _cancel_node(self, msg):
        """ Set a node to the cancel state
            @param msg [String] The id of the node to cancel
        """
        if msg.data in self.node_map:
            rospy.loginfo("Canceling " + msg.data)
            self.node_map[msg.data]._cancel()

    def update(self):
        """ Update the tree root and publish status
        """
        self.parent.tick()
        self.publish_status()
