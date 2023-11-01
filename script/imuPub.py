#!/home/aduragbemi/.pyenv/shims/python

import random
import time

import rospy
from std_msgs.msg import String

from sensor_msgs.msg import Imu

# Function to generate synthetic IMU data
def generate_imu_data():
    imu_data = {
        "accelerometer": {
            "x": random.uniform(-1.0, 1.0),
            "y": random.uniform(-1.0, 1.0),
            "z": random.uniform(-1.0, 1.0),
        },
        "gyroscope": {
            "x": random.uniform(-1.0, 1.0),
            "y": random.uniform(-1.0, 1.0),
            "z": random.uniform(-1.0, 1.0),
        },
        "magnetometer": {
            "x": random.uniform(-100.0, 100.0),
            "y": random.uniform(-100.0, 100.0),
            "z": random.uniform(-100.0, 100.0),
        },
        "temperature": random.uniform(20.0, 40.0),
    }
    return imu_data


def simulateImu(sensor_type):
    imu_data = generate_imu_data()
    print(sensor_type)
    print("imu {")
    print(f"  accelerometer {{")
    print(f"    x: {imu_data['accelerometer']['x']}")
    print(f"    y: {imu_data['accelerometer']['y']}")
    print(f"    z: {imu_data['accelerometer']['z']}")
    print(f"  }}")
    print(f"  gyroscope {{")
    print(f"    x: {imu_data['gyroscope']['x']}")
    print(f"    y: {imu_data['gyroscope']['y']}")
    print(f"    z: {imu_data['gyroscope']['z']}")
    print(f"  }}")
    print(f"  magnetometer {{")
    print(f"    x: {imu_data['magnetometer']['x']}")
    print(f"    y: {imu_data['magnetometer']['y']}")
    print(f"    z: {imu_data['magnetometer']['z']}")
    print(f"  }}")
    print(f"  temperature: {imu_data['temperature']}")
    print("}")
    print("\n")
    time.sleep(1)  # Simulate data being generated every second


# for sensor_type in ["Raw Imu1Tel", "Raw Imu2Tel", "CalibratedImuTel"]:

def talker():
    #create a new publisher. we specify the topic name, then type of message then the queue size
    pub = rospy.Publisher('/blueye_x3/Imu', Imu, queue_size=10)
    #we need to initialize the node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node 
    rospy.init_node('talker', anonymous=True)
    #set the loop rate
    rate = rospy.Rate(1) # 1hz
    #keep publishing until a Ctrl-C is pressed
    i = 0
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % i
        rospy.loginfo(hello_str)
        pub.publish(simulateImu("Raw Imu1Tel"))
        rate.sleep()
        i=i+1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
