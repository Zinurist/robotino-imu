import rospy
import sys
import imu_t as im
#import imu_o as im

"""
Test for the imu reader.
"""

def imu_test(hertz=50, secs=5):
    rospy.init_node('imu_tester', anonymous=True)
    rate = rospy.Rate(hertz)
    count = 0
    #empty queue
    im.get_vals()
    for i in range(secs*hertz):
        val = im.get_vals()
        print('Received %s' % len(val))
        count += len(val)
        #print(val['accel'])
        #print(val['gyro'])
        #print(val['compass'])
        print('-----------------------------------------')
        rate.sleep()
    print('Total: %s' % count)
    print('Rate: %s' % (count/secs))
    im.stop()

if __name__ == '__main__':
    try:
        imu_test()
    except rospy.ROSInterruptException:
        pass

