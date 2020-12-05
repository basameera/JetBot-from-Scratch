#!/usr/bin/env python
# usage: rosrun jetbot_hw ina_pub.py
import rospy
from std_msgs.msg import String
from ina219 import INA219
from ina219 import DeviceRangeError

ROS_TOPIC = 'jetbot_hw/ina'
SHUNT_OHMS = 0.1


def read_ina(ina):
    out_str = '--empty--'
    try:
        V_BUS = ina.voltage()
        I_BUS = ina.current()
        P_BUS = ina.power()
        V_SHUNT = ina.shunt_voltage()

        out_str = 'Voltage: {:.3f} V | Current: {:.3f} mA | Power: {:.3f} mW | V Shunt: {:.3f} mV'.format(
            V_BUS, I_BUS, P_BUS, V_SHUNT)

    except DeviceRangeError as e:
        # Current out of device range with specified shunt resistor
        print(e)
    return out_str


def publisher():
    # setup ros node
    pub = rospy.Publisher(ROS_TOPIC, String, queue_size=10)
    rospy.init_node('ina_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 10hz

    # setup ina
    """
    INA219(shunt_ohms, max_expected_amps=None,
                    busnum=None, address=__ADDRESS,
                    log_level=logging.ERROR)
    """
    ina = INA219(SHUNT_OHMS, busnum=1, address=0x41)
    ina.configure()

    while not rospy.is_shutdown():

        hello_str = read_ina(ina)

        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
