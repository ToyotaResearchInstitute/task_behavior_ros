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

from rospy_tutorials.srv import AddTwoInts
from rospy_tutorials.srv import AddTwoIntsResponse

from task_behavior_engine.tree import Blackboard
from task_behavior_engine.tree import NodeData
from task_behavior_engine.tree import NodeStatus
from task_behavior_ros import service


class AddTwoIntsService():

    def __init__(self, name):
        rospy.Service(
            name, AddTwoInts, self.handle_add_two_ints)

    def handle_add_two_ints(self, req):
        print "Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b))
        return AddTwoIntsResponse(req.a + req.b)


def setup_module():
    AddTwoIntsService('test_service')


class TestServiceClient(object):

    def test_server_connected(self):
        # test nominal condition
        sc = service.ServiceClient(name="service_tutorial_client",
                                   service_name="test_service",
                                   service_type=AddTwoInts)
        sc.config(NodeData())
        assert_equal(sc.server_connected, True)

        # test server not available
        sc = service.ServiceClient(name="test_no_server",
                                   service_name="no_server",
                                   service_type=AddTwoInts,)
        sc.config(NodeData())
        assert_equal(sc.server_connected, False)

    def test_request(self):
        # test nominal condition
        blackboard = Blackboard()
        request = (1, 2)
        sc = service.ServiceClient(name="service_tutorial_client",
                                   service_name="test_service",
                                   service_type=AddTwoInts,
                                   request=request,
                                   blackboard=blackboard)
        result = sc.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal(3, blackboard.get("result", sc._id).sum)

        # test wrong request type
        request = "wrong"
        sc = service.ServiceClient(name="service_tutorial_client",
                                   service_name="test_service",
                                   service_type=AddTwoInts,
                                   request=request,
                                   blackboard=blackboard)
        result = sc.tick()
        assert_equal(result.status, NodeStatus.FAIL)

    def test_request_cb(self):
        def request_cb(nodedata):
            return (1, 2)

        def wrong_request_cb(nodedata):
            return "wrong"

        # test nominal condition
        blackboard = Blackboard()
        sc = service.ServiceClient(name="service_tutorial_client",
                                   service_name="test_service",
                                   service_type=AddTwoInts,
                                   request_cb=request_cb,
                                   blackboard=blackboard)
        result = sc.tick()
        assert_equal(result.status, NodeStatus.SUCCESS)
        assert_equal(3, blackboard.get("result", sc._id).sum)

        # test wrong request type
        sc = service.ServiceClient(name="service_tutorial_client",
                                   service_name="test_service",
                                   service_type=AddTwoInts,
                                   request_cb=wrong_request_cb,
                                   blackboard=blackboard)
        result = sc.tick()
        assert_equal(result.status, NodeStatus.FAIL)

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
