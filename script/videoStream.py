import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoStream:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)

    def start_stream(self):
        pipeline = (
            "rtspsrc location=rtsp://192.168.1.101:8554/test latency=0 ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        )
        # pipeline = (
        #     "videotestsrc ! videoconvert ! appsink"
        # )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            rospy.logerr("Error: Unable to open video stream")
            return

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("Warning: Unable to read frame")
                continue

            try:
                image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(image_msg)
            except Exception as e:
                rospy.logerr(f"Error: {e}")

        cap.release()

if __name__ == '__main__':
    rospy.init_node('video_stream')
    stream = VideoStream()
    stream.start_stream()
