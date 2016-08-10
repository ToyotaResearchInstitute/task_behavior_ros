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


import rospy
import sys

from std_msgs.msg import String

from task_behavior_engine import branch
from task_behavior_engine import tree

from task_behavior_ros import introspection
from task_behavior_ros import time
from task_behavior_ros import topic


def timed_publish(name, msg, timeout):
    ''' Publish a std_msg/String message after timeout
        @param name [str] The name of the behavior
        @param msg [str] The message to publish
        @param timeout [float] The time (in seconds) to wait to publish
    '''
    timer = time.TimedWait(name="pub wait", timeout=timeout)
    message = String()
    message.data = msg
    publish = topic.TopicPublisher(
        name="publish", topic_name="test", topic_type=String, msg=message)

    publish_timer = branch.Sequencer(name)
    publish_timer.add_child(timer)
    publish_timer.add_child(publish)

    return publish_timer

if __name__ == '__main__':
    rospy.init_node('example')
    rospy.loginfo("started")

    # Create the timed_publish behavior.
    behavior = timed_publish(name="example", msg="hello world", timeout=1.0)

    # Run any cancel hooks on shutdown (ctrl+c).
    rospy.on_shutdown(behavior._cancel)
    # Set up the inspection server to publish current status.
    ros_status = introspection.Introspection(behavior)

    # Set up repeating rate to tick the behavior.
    r = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            # Tick the behavior.
            result = behavior.tick()
            # Publish the status to the introspection server.
            ros_status.publish_status()
            rospy.loginfo("behavior result: " + str(result))

            if not result == tree.NodeStatus.ACTIVE:
                rospy.logwarn("finised with status: " + str(result))
            r.sleep()
    except:
        e = sys.exc_info()[0]
        rospy.logerr(e)
