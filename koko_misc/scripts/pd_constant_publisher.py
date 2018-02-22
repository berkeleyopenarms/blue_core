#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float64MultiArray

def main():
    p_constants = [25.0, 25.0, 15.0, 15.0, 10.0, 5.0, 5.0]
    # p_constants = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    d_constants = [3.0, 4.0, 3.0, 2.0, 2.0, 1.0, 1.0]

    p_constants = [10.0, 10.0, 10.0, 5.0, 5.0, 3.0, 3.0]
    #d_constants = [3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 1.0]

    #p_constants = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #d_constants = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]

    rospy.init_node('pd_publisher', anonymous=True)
    rate = rospy.Rate(0.1)

    p_publisher = rospy.Publisher("p_terms", Float64MultiArray, queue_size=1)
    d_publisher = rospy.Publisher("d_terms", Float64MultiArray, queue_size=1)

    for i in range(2):
        p_terms = Float64MultiArray()
        d_terms = Float64MultiArray()

        p_terms.data = p_constants
        d_terms.data = d_constants

        p_publisher.publish(p_terms)
        d_publisher.publish(d_terms)
        print('finished publishing new PD constants')
        time.sleep(5)

if __name__ == '__main__':
    main()
