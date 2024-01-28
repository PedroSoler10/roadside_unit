#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String

class ScenarioClassificator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('scenario_classification')

        # Create a Subscriber for the ball_detection topic
        self.subscriber = rospy.Subscriber('ball_detection', Bool, self.callback_ball_detection)

        # Create a Publisher for the scenario topic
        self.publisher = rospy.Publisher('scenario', String, queue_size=10)

    def callback_ball_detection(self, data):
        # Process the incoming boolean message and determine the scenario
        if data.data:
            scenario = "used"
        else:
            scenario = "free"

        # Publish the scenario
        self.publisher.publish(scenario)
        rospy.loginfo("Published: %s", scenario)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        
        node = ScenarioClassificator()
        node.run()
    except rospy.ROSInterruptException:
        pass
