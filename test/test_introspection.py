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


from nose.tools import assert_equal
from nose.tools import assert_not_equal
from nose.tools import assert_raises

import rospy
from std_msgs.msg import String

from task_behavior_engine.tree import Node
from task_behavior_engine.tree import NodeStatus
from task_behavior_engine.branch import Sequencer
from task_behavior_engine.decorator import Repeat

from task_behavior_msgs.msg import TreeStructure
from task_behavior_msgs.msg import TreeNode
from task_behavior_msgs.msg import TreeStatus
from task_behavior_msgs.msg import TreeNodeStatus

from task_behavior_ros import introspection


class Continue(Node):

    def __init__(self, name, *args, **kwargs):
        super(Continue, self).__init__(name, run_cb=self.run, *args, **kwargs)

    def run(self, nodedata):
        return NodeStatus(NodeStatus.ACTIVE)


class TestIntrospection(object):

    def setUp(self):
        CONTINUE = Continue(name="CONTINUE")
        CONTINUE2 = Continue(name="CONTINUE2")
        LEVEL1 = Sequencer(name="LEVEL1")
        LEVEL2 = Repeat(name="LEVEL2")
        LEVEL2.set_child(CONTINUE)
        LEVEL1.add_child(LEVEL2)
        LEVEL1.add_child(CONTINUE2)

        self.id_name_map = {}
        self.id_name_map[str(LEVEL1._id)] = LEVEL1._name
        self.id_name_map[str(LEVEL2._id)] = LEVEL2._name
        self.id_name_map[str(CONTINUE._id)] = CONTINUE._name
        self.id_name_map[str(CONTINUE2._id)] = CONTINUE2._name

        self.test = introspection.Introspection(LEVEL1)

    def test_structure_pub(self):
        self.test._reload()
        msg = rospy.wait_for_message('tree/structure', TreeStructure)

        assert_equal(len(msg.node), 4)
        valid_names = [self.id_name_map[key] for key in self.id_name_map]

        for node in msg.node:
            print "checking: " + node.name
            assert_equal(node.name in valid_names, True)
            valid_names.remove(node.name)
            if node.name == "LEVEL1":
                assert_equal(node.type, TreeNode.BEHAVIOR)
                assert_equal(len(node.children), 2)
            if node.name == "LEVEL2":
                assert_equal(node.type, TreeNode.DECORATOR)
                assert_equal(len(node.children), 1)
            if node.name == "CONTINUE":
                assert_equal(node.type, TreeNode.NODE)
                assert_equal(len(node.children), 0)
            if node.name == "CONTINUE2":
                assert_equal(node.type, TreeNode.NODE)
                assert_equal(len(node.children), 0)
        assert_equal(len(valid_names), 0)

        print "test second call"
        self.test.publish_structure()

        msg = rospy.wait_for_message('tree/structure', TreeStructure)

        assert_equal(len(msg.node), 4)
        valid_names = [self.id_name_map[key] for key in self.id_name_map]

        for node in msg.node:
            print "checking: " + node.name
            assert_equal(node.name in valid_names, True)
            valid_names.remove(node.name)
            if node.name == "LEVEL1":
                assert_equal(node.type, TreeNode.BEHAVIOR)
                assert_equal(len(node.children), 2)
            if node.name == "LEVEL2":
                assert_equal(node.type, TreeNode.DECORATOR)
                assert_equal(len(node.children), 1)
            if node.name == "CONTINUE":
                assert_equal(node.type, TreeNode.NODE)
                assert_equal(len(node.children), 0)
            if node.name == "CONTINUE2":
                assert_equal(node.type, TreeNode.NODE)
                assert_equal(len(node.children), 0)
        assert_equal(len(valid_names), 0)

    def test_status_pub(self):
        self.test._reload()
        self.test.publish_status()
        msg = rospy.wait_for_message('tree/status', TreeStatus)

        assert_equal(len(msg.id), 4)
        assert_equal(len(msg.status), 4)
        valid_keys = self.id_name_map.keys()

        for i, scope in enumerate(msg.id):
            print "checking: " + scope
            print "against: ", valid_keys
            assert_equal(scope in valid_keys, True)
            valid_keys.remove(scope)
            print "verify " + scope + " initialzed to PENDING"
            assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)
        assert_equal(len(valid_keys), 0)

        print "running the parent"
        self.test.parent.tick()
        self.test.publish_status()
        msg = rospy.wait_for_message('tree/status', TreeStatus)
        valid_keys = self.id_name_map.keys()

        for i, scope in enumerate(msg.id):
            print "checking: " + scope
            print "against: ", valid_keys
            assert_equal(scope in valid_keys, True)
            valid_keys.remove(scope)
            if self.id_name_map[scope] == "LEVEL1":
                assert_equal(msg.status[i].status, TreeNodeStatus.ACTIVE)
            if self.id_name_map[scope] == "LEVEL2":
                assert_equal(msg.status[i].status, TreeNodeStatus.ACTIVE)
            if self.id_name_map[scope] == "CONTINUE":
                assert_equal(msg.status[i].status, TreeNodeStatus.ACTIVE)
            if self.id_name_map[scope] == "CONTINUE2":
                assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)

    def test_force(self):
        node = self.test.parent._children[0]
        assert_equal(node._name, "LEVEL2")

        self.test.update()
        msg = TreeStatus()
        msg.id.append(str(node._id))
        status = TreeNodeStatus()
        status.status = TreeNodeStatus.FAIL
        status.text = "forced fail"
        msg.status.append(status)
        print "setting force message: " + str(msg)
        self.test._force_node(msg)

        self.test.update()
        msg = self.test._create_status_msg(self.test.parent, TreeStatus())
        print msg
        valid_keys = self.id_name_map.keys()
        for i, scope in enumerate(msg.id):
            print "checking: " + scope
            print "against: ", valid_keys
            assert_equal(scope in valid_keys, True)
            valid_keys.remove(scope)
            if self.id_name_map[scope] == "LEVEL1":
                assert_equal(msg.status[i].status, TreeNodeStatus.FAIL)
            if self.id_name_map[scope] == "LEVEL2":
                assert_equal(msg.status[i].status, TreeNodeStatus.FAIL)
            if self.id_name_map[scope] == "CONTINUE":
                assert_equal(msg.status[i].status, TreeNodeStatus.CANCEL)
            if self.id_name_map[scope] == "CONTINUE2":
                assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)

    def test_cancel(self):
        node = self.test.parent._children[0]
        assert_equal(node._name, "LEVEL2")

        self.test.update()
        msg = TreeStatus()
        msg.id = []
        msg.id.append(str(node._id))
        msg.status = []
        status = TreeNodeStatus()
        status.status = TreeNodeStatus.CANCEL
        status.text = "forced cancel"
        msg.status.append(status)
        print "setting force message: " + str(msg)
        self.test._force_node(msg)

        self.test.update()
        msg = self.test._create_status_msg(self.test.parent, TreeStatus())
        print msg
        valid_keys = self.id_name_map.keys()
        for i, scope in enumerate(msg.id):
            print "checking: " + scope
            print "against: ", valid_keys
            assert_equal(scope in valid_keys, True)
            valid_keys.remove(scope)
            if self.id_name_map[scope] == "LEVEL1":
                assert_equal(msg.status[i].status, TreeNodeStatus.FAIL)
            if self.id_name_map[scope] == "LEVEL2":
                assert_equal(msg.status[i].status, TreeNodeStatus.CANCEL)
            if self.id_name_map[scope] == "CONTINUE":
                assert_equal(msg.status[i].status, TreeNodeStatus.CANCEL)
            if self.id_name_map[scope] == "CONTINUE2":
                assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)

    def test_run(self):
        self.test.publish_status()
        msg = rospy.wait_for_message('tree/status', TreeStatus)
        valid_keys = self.id_name_map.keys()

        for i, scope in enumerate(msg.id):
            print "checking: " + scope
            print "against: ", valid_keys
            assert_equal(scope in valid_keys, True)
            valid_keys.remove(scope)
            if self.id_name_map[scope] == "LEVEL1":
                assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)
            if self.id_name_map[scope] == "LEVEL2":
                assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)
            if self.id_name_map[scope] == "CONTINUE":
                assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)
            if self.id_name_map[scope] == "CONTINUE2":
                assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)

        msg = String()
        msg.data = str(self.test.parent._id)
        self.test._tick_node(msg)
        self.test.publish_status()
        msg = rospy.wait_for_message('tree/status', TreeStatus)
        valid_keys = self.id_name_map.keys()

        for i, scope in enumerate(msg.id):
            print "checking: " + scope
            print "against: ", valid_keys
            assert_equal(scope in valid_keys, True)
            valid_keys.remove(scope)
            if self.id_name_map[scope] == "LEVEL1":
                assert_equal(msg.status[i].status, TreeNodeStatus.ACTIVE)
            if self.id_name_map[scope] == "LEVEL2":
                assert_equal(msg.status[i].status, TreeNodeStatus.ACTIVE)
            if self.id_name_map[scope] == "CONTINUE":
                assert_equal(msg.status[i].status, TreeNodeStatus.ACTIVE)
            if self.id_name_map[scope] == "CONTINUE2":
                assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)
