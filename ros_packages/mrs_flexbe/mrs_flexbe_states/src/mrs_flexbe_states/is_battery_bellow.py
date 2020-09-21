#!/usr/bin/env python
import rospy

from flexbe_core.proxy.proxy_service_caller import ProxyServiceCaller
from flexbe_core.proxy.proxy_subscriber_cached import ProxySubscriberCached
from flexbe_core import EventState, Logger
import sensor_msgs.msg
import inspect
import rostopic



class IsBatteryBellowValueState(EventState):
    '''
    Checking if battery is bellow something

    -- battery_topic     string     topic of battery data

    <= continue             Given time has passed.
    <= failed                 Example for a failure outcome.

    '''

    def __init__(self, battery_status_topic, num_battery_cells):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(IsBatteryBellowValueState, self).__init__(input_keys=['cell_voltage_target'],
                                                        output_keys=['current_cell_voltage'],
                                                        outcomes=['is_bellow', 'is_ok']
                                                    
                                                    )

        self._topic = rospy.resolve_name(battery_status_topic)
        self._num_battery_cells = num_battery_cells
        # Store state parameter for later use.
        
        (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)
        #Logger.loginfo('[IsBatteryBellowValueState]: topic %s msg_topic %s'%(self._topic,msg_topic))
        if msg_topic == self._topic:
            msg_type = self._get_msg_from_path(msg_path)
            self._sub_battery = ProxySubscriberCached({self._topic: msg_type})
            self._connected = True
            Logger.loginfo('[IsBatteryBellowValueState]: Successfully subscribed to topic %s' % self._topic)
            #brick_subs_ = rospy.Subscriber(self._topic, sensor_msgs.msg.BatteryState, self.battery_status_callback)

        self.moving_average_hist_v_param = 0.1
        self.volage_per_cell = 4.2
        self.voltage = num_battery_cells * self.volage_per_cell
        
        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._start_time = None

    


    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        # If no outcome is returned, the state will stay active.
        """
        sensor_msgs/BatteryState
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
        
        
        
        if self._sub_battery.has_msg(self._topic):
            
            message = self._sub_battery.get_last_msg(self._topic)
            Logger.loginfo('[IsBatteryBellowValueState]: received voltage per cell %.2f' % (message.voltage/ float(self._num_battery_cells)))
            self._sub_battery.remove_last_msg(self._topic)
            #print('[IsBatteryBellowValueState] message received')
            
            if self.voltage is None:
                self.voltage = message.voltage
            
            cell_voltage_target = userdata.cell_voltage_target
            self.voltage = self.voltage * self.moving_average_hist_v_param + (1.0 - self.moving_average_hist_v_param) * message.voltage
            self.volage_per_cell = self.voltage / float(self._num_battery_cells)
            userdata.current_cell_voltage = self.volage_per_cell
            rospy.loginfo_throttle(20,('[IsBatteryBellowValueState] voltage per cell is %f'%(self.volage_per_cell)))
        
            if self.volage_per_cell < cell_voltage_target:
	            Logger.loginfo('[IsBatteryBellowValueState] voltage per cell %f is bellow set threshold %f'%(self.volage_per_cell,cell_voltage_target))
	            return 'is_bellow'
            else:
	            Logger.loginfo('[IsBatteryBellowValueState] voltage per cell %f is above set threshold %f'%(self.volage_per_cell,cell_voltage_target))
	            return 'is_ok'
            
    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
        # It is primarily used to start actions which are associated with this state.

        Logger.loginfo('[IsBatteryBellowValueState] entering state')
        if not self._connected:
            (msg_path, msg_topic, fn) = rostopic.get_topic_type(self._topic)
            if msg_topic == self._topic:
                msg_type = self._get_msg_from_path(msg_path)
                self._sub_battery = ProxySubscriberCached({self._topic: msg_type})
                self._connected = True
                Logger.loginfo('[IsBatteryBellowValueState]: Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('[IsBatteryBellowValueState]: Topic %s still not available, giving up.' % self._topic)

            
        if self._connected and self._sub_battery.has_msg(self._topic):
            Logger.loginfo('[IsBatteryBellowValueState]: Waiting for msg from topic %s' % self._topic)
            self._sub_battery.remove_last_msg(self._topic)
   
    def _get_msg_from_path(self, msg_path):
        msg_import = msg_path.split('/')
        msg_module = '%s.msg' % (msg_import[0])
        package = __import__(msg_module, fromlist=[msg_module])
        clsmembers = inspect.getmembers(package, lambda member: inspect.isclass(member) and member.__module__.endswith(msg_import[1]))
        return clsmembers[0][1]
