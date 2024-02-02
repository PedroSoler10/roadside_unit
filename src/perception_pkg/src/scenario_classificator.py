#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D 

class ScenarioClassificator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('scenario_classificator')

        detections_topic = rospy.get_param('detections_topic', '/detectnet/detections')
        self.visualization_flag = rospy.get_param('scenario_visualization_flag', True)

        # Create a Publisher for the scenario topic
        self.publisher = rospy.Publisher('/scenario', String, queue_size=10)

        # Init thresholds
        self.image_resolution = 720.0*1280.0    # 720*1080=777600
        up_t_i = 0.3
        low_t_i = 0.2
        up_t_c = 0.8
        low_t_c = 0.5
        self.upper_importance_threshold = up_t_i * up_t_c       # 0.3*0.8=0.24
        self.lower_importance_threshold = low_t_i * low_t_c     # 0.2*0.5=0.1

        # Init metrics
        self.time, self.width, self.height, self.size, self.conf, self.imp = [], [], [], [], [], []
        

        self.scenario = "unknown"
        # Create a Subscriber for the detections topic
        self.subscriber = rospy.Subscriber('/detectnet/detections', Detection2DArray, self.callback)

    def visualize_metrics(self, detection, stamp):
        self.time.append(stamp)
        self.width.append(detection.bbox.size_x)
        self.height.append(detection.bbox.size_y)
        self.size.append(detection.bbox.size_x * detection.bbox.size_y)
        self.conf.append(detection.results[0].score)
        self.imp.append(detection.bbox.size_x * detection.bbox.size_y * detection.results[0].score / self.image_resolution)
        print "time:\t{:.2f}\twidth:\t{:.0f}\theight:\t{:.0f}\tsize:\t{:.0f}\tconf:\t{:.4f}\timp:\t{:.4f}".format(self.time[-1], self.width[-1], self.height[-1], self.size[-1], self.conf[-1], self.imp[-1])


    def callback(self, data):
        # Process the incoming boolean message and determine the scenario
        # if data.data:
        #     scenario = "used"
        # else:
        #     scenario = "free"

        main_human_detection = Detection2D()
        max_importance = 0.0

        for detection in data.detections:
            if detection.results[0].id == 1:
                print("Human detected")
                importance = detection.bbox.size_x * detection.bbox.size_y * detection.results[0].score / self.image_resolution
                if importance > max_importance:
                    print("Pedestrian detected")
                    max_importance = importance
                    main_human_detection = detection

        if max_importance > self.upper_importance_threshold:
            self.scenario = "used"
        elif max_importance > self.lower_importance_threshold:
            self.scenario = "solicited"
        else:
            self.scenario = "free"

        # Publish the scenario
        self.publisher.publish(self.scenario)
        rospy.loginfo("Published: %s", self.scenario)

        if self.visualization_flag and self.scenario != "free":
            self.visualize_metrics(main_human_detection, data.header.stamp.to_sec())
            
    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        
        node = ScenarioClassificator()
        node.run()
    except rospy.ROSInterruptException:
        pass
