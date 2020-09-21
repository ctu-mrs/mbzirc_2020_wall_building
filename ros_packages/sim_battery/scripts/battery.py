#!/usr/bin/env python
import rospy
import math
import sys



import sensor_msgs.msg
import mavros_msgs.msg

#sensor_msgs.msg.BatteryState

class BatterySimulation:

    def __init__(self):
        rospy.init_node('sim_battery')
        rospy.loginfo("[BatterySimulation] battery simulation init")
        
        self.battery_pub = rospy.Publisher('~battery', sensor_msgs.msg.BatteryState, queue_size=10)
        self.thrust_sub = rospy.Subscriber('~thrust_in',mavros_msgs.msg.AttitudeTarget,callback=self.thrust_callback)
        
        self.thrust = 0
 

        self.n_cells = rospy.get_param('~n_cells')
        rospy.loginfo("self.n_cells: %s" % (str(self.n_cells)))
        self.max_cell_voltage = rospy.get_param('~max_cell_voltage')
        rospy.loginfo("self.max_cell_voltage: %s" % (str(self.max_cell_voltage)))
        self.min_cell_voltage = rospy.get_param('~min_cell_voltage')
        rospy.loginfo("self.min_cell_voltage: %s" % (str(self.min_cell_voltage)))
        self.bat_capacity_s = rospy.get_param('~bat_capacity_s')
        rospy.loginfo("self.bat_capacity_s: %s" % (str(self.bat_capacity_s)))

        self.cell_voltage = self.max_cell_voltage
        self.publish_rate = 10
    
    def thrust_callback(self,data):
        self.thrust = data.thrust
   
    def run(self):
        """spin the wheel"""
        """
            uint8 POWER_SUPPLY_STATUS_UNKNOWN=0
            uint8 POWER_SUPPLY_STATUS_CHARGING=1
            uint8 POWER_SUPPLY_STATUS_DISCHARGING=2
            uint8 POWER_SUPPLY_STATUS_NOT_CHARGING=3
            uint8 POWER_SUPPLY_STATUS_FULL=4
            uint8 POWER_SUPPLY_HEALTH_UNKNOWN=0
            uint8 POWER_SUPPLY_HEALTH_GOOD=1
            uint8 POWER_SUPPLY_HEALTH_OVERHEAT=2
            uint8 POWER_SUPPLY_HEALTH_DEAD=3
            uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE=4
            uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE=5
            uint8 POWER_SUPPLY_HEALTH_COLD=6
            uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE=7
            uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE=8
            uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN=0
            uint8 POWER_SUPPLY_TECHNOLOGY_NIMH=1
            uint8 POWER_SUPPLY_TECHNOLOGY_LION=2
            uint8 POWER_SUPPLY_TECHNOLOGY_LIPO=3
            uint8 POWER_SUPPLY_TECHNOLOGY_LIFE=4
            uint8 POWER_SUPPLY_TECHNOLOGY_NICD=5
            uint8 POWER_SUPPLY_TECHNOLOGY_LIMN=6
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            float32 voltage
            float32 current
            float32 charge
            float32 capacity
            float32 design_capacity
            float32 percentage
            uint8 power_supply_status
            uint8 power_supply_health
            uint8 power_supply_technology
            bool present
            float32[] cell_voltage
            string location
            string serial_number
        """
        rospy.loginfo("[BatterySimulation] starting battery simulation")
        r = rospy.Rate(self.publish_rate) # 10hz
        discharge_rate = (self.max_cell_voltage-self.min_cell_voltage) / float(self.bat_capacity_s*self.publish_rate)
        while not rospy.is_shutdown():
           if self.thrust > 0.1:
               self.cell_voltage -= discharge_rate
               rospy.loginfo_throttle(5,"[BatterySimulation] battery voltage is %f"%(self.cell_voltage))
               self.cell_voltage = max(self.cell_voltage,self.min_cell_voltage)

           state = sensor_msgs.msg.BatteryState()
           state.header.stamp = rospy.Time.now() 
           state.voltage = self.cell_voltage * float(self.n_cells)
           state.cell_voltage = self.n_cells*[self.cell_voltage]

           self.battery_pub.publish(state)
           r.sleep()

        rospy.spin()


if __name__ == '__main__':
    battery = BatterySimulation()
    battery.run()
