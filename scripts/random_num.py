#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import random

def publish_random_numbers():
    # '/random_numbers' 토픽에 Int32 메시지를 발행하는 퍼블리셔를 초기화합니다.
    pub = rospy.Publisher('/random_numbers', Int32, queue_size=10)
    rospy.init_node('random_number_publisher', anonymous=True)
    rate = rospy.Rate(1) # 초당 1회 발행

    for i in range(50):
        if rospy.is_shutdown():
            break
        # 1부터 100까지의 수 중에서 무작위로 하나를 선택합니다.
        random_number = random.randint(1, 100)
        rospy.loginfo("Publishing: %s", random_number)
        pub.publish(random_number)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_random_numbers()
    except rospy.ROSInterruptException:
        pass

