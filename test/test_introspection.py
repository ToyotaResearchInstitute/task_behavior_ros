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
from nose.tools import assert_not_equal
from nose.tools import assert_raises
from nose.tools import assert_true

from std_msgs.msg import String

from task_behavior_engine.tree import Blackboard
from task_behavior_engine.tree import Node
from task_behavior_engine.tree import NodeStatus
from task_behavior_engine.branch import Sequencer
from task_behavior_engine.decorator import Repeat

from task_behavior_msgs.msg import NodeData
from task_behavior_msgs.msg import TreeDataDump
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

        CONTINUE._blackboard.save('foo', 1, CONTINUE._id)
        CONTINUE2._blackboard.save('bar', 2, CONTINUE2._id)
        LEVEL1._blackboard.save('toy', 1.0, LEVEL1._id)
        LEVEL2._blackboard.save('yoda', 2.0, LEVEL2._id)

        self.id_name_map = {}
        self.id_name_map[str(LEVEL1._id)] = LEVEL1._name
        self.id_name_map[str(LEVEL2._id)] = LEVEL2._name
        self.id_name_map[str(CONTINUE._id)] = CONTINUE._name
        self.id_name_map[str(CONTINUE2._id)] = CONTINUE2._name

        self.test = introspection.Introspection(LEVEL1)

    def test_structure_pub(self):
        self.test._reload()
        msg = rospy.wait_for_message('tree/structure', TreeStructure)
        self.check_structure_msg(msg)

        print "test second call"
        self.test.publish_structure()
        msg = rospy.wait_for_message('tree/structure', TreeStructure)
        self.check_structure_msg(msg)

       
        
    def check_structure_msg(self, msg):
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
        self.check_status_msg_pending(msg)
        
        print "running the parent"
        self.test.parent.tick()
        self.test.publish_status()
        msg = rospy.wait_for_message('tree/status', TreeStatus)
        self.check_status_msg_ticked(msg)

        
                
    def check_status_msg_pending(self, msg):
        #check status
        assert_equal(len(msg.id), 4)
        assert_equal(len(msg.name), 4)
        assert_equal(len(msg.status), 4)
        assert_equal(len(msg.data), 4)
        valid_keys = self.id_name_map.keys()

        for i, scope in enumerate(msg.id):
            print "checking: " + scope
            print "against: ", valid_keys
            assert_equal(scope in valid_keys, True)
            valid_keys.remove(scope)
            print "verify " + scope + " initialzed to PENDING"
            assert_equal(msg.status[i].status, TreeNodeStatus.PENDING)
        assert_equal(len(valid_keys), 0)
        
        #check nodedata
        for data in msg.data:
            assert_equal(len(data.key), 1)
            assert_equal(len(data.value), 1)

        assert_true('toy' in msg.data[0].key)
        assert_true('1.0' in msg.data[0].value)
        assert_true('yoda' in msg.data[1].key)
        assert_true('2.0' in msg.data[1].value)
        assert_true('foo' in msg.data[2].key)
        assert_true('1' in msg.data[2].value)
        assert_true('bar' in msg.data[3].key)
        assert_true('2' in msg.data[3].value)

    def check_status_msg_ticked(self, msg):
        #check status    
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
         
        #check nodedata
        for i in range(len(msg.data) - 1):
            assert_equal(len(msg.data[i].key), 2)
            assert_equal(len(msg.data[i].value), 2)
        assert_equal(len(msg.data[3].key), 1)
        assert_equal(len(msg.data[3].value), 1)

        assert_true('toy' in msg.data[0].key)
        assert_true('1.0' in msg.data[0].value)
        assert_true('yoda' in msg.data[1].key)
        assert_true('2.0' in msg.data[1].value)
        assert_true('foo' in msg.data[2].key)
        assert_true('1' in msg.data[2].value)
        assert_true('bar' in msg.data[3].key)
        assert_true('2' in msg.data[3].value)
    
        

    def test_data_dump_pub(self):
        self.test._reload()
        self.test.publish_data_dump()
        msg = rospy.wait_for_message('tree/data', TreeDataDump)
        self.check_structure_msg(msg.structure)
        self.check_status_msg_pending(msg.status)
        
        print "running the parent"
        self.test.parent.tick()
        self.test.publish_data_dump()
        msg = rospy.wait_for_message('tree/data', TreeDataDump)
        
        self.check_structure_msg(msg.structure)
        self.check_status_msg_ticked(msg.status)
        

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

if __name__ == '__main__':
    # This code will run the test in this file.'
    module_name = sys.modules[__name__].__file__

    parser = argparse.ArgumentParser(description='Perform unit test.')
    parser.add_argument(
        '--gtest_output', nargs='?', default='test.xml')

    args, unknown = parser.parse_known_args()

    noseargs = [sys.argv[0], module_name, '--with-xunit',
                '--xunit-file=' + str(args.gtest_output.lstrip('xml:'))]
    nose.run(argv=noseargs)
