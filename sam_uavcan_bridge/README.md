uavcan_ros_bridge
=================

Bridge for communication between ROS and the Dronecan CAN-bus protocol

## Dependencies & building

After cloning, execute the following command in the cloned repo:
```
git submodule update --init --recursive
```

## Generate files
To generate the dsdl message files use the following command in the dsdl directory:
```
python3 dronecan_dsdlc/dronecan_dsdlc.py -O dsdl_generated DSDL/uavcan smarc_uavcan_messages smarc_uavcan_services

```
## Usage

Launch conversion in both directions (between uavcan and ros) by running the launch file:
```
ros2 launch sam_uavcan_bridge bridge.launch
```
This will launch two nodes: the `uavcan_to_ros_bridge_node` and the `ros_to_uavcan_bridge_node`
which handle the conversion in the respective directions. Any new type conversions should
be added to the node that handles the relevant direction.

## Adding new conversions

You can add new conversion headers for ros to uavcan and uavcan to ros with corresponding implementations in the src folder. An example implementation for an IMU might look like this:
```cpp
template <>
bool convert(const uavcan_equipment_ahrs_RawIMU& uav_msg, std::shared_ptr<sensor_msgs::msg::Imu> ros_msg)
{
    ros_msg->header.stamp = convert_timestamp(uav_msg.timestamp);
    ros_msg->linear_acceleration.x = uav_msg.accelerometer_latest[0];
    ros_msg->linear_acceleration.y = uav_msg.accelerometer_latest[1];
    ros_msg->linear_acceleration.z = uav_msg.accelerometer_latest[2];
    ros_msg->angular_velocity.x = uav_msg.rate_gyro_latest[0];
    ros_msg->angular_velocity.y = uav_msg.rate_gyro_latest[1];
    ros_msg->angular_velocity.z = uav_msg.rate_gyro_latest[2];
    return true;
}
```
You then need to add them to the build configuration in the cmake file and link them into one of the bridges, by adding them to one of the respective libraries.

## Testing conversions
To test converison between ros2 and Dronecan one can use virtual can, run the script to start vcan:
```
./setup_vcan.sh
```
## Existing conversions (*work in progress*)

### ROS to UAVCAN
* `std_msgs/Float32` to `uavcan.equipment.actuator.ArrayCommand` on ros topic `/uavcan_command`
* `sam_msgs/ArrayCommand` to `uavcan.equipment.actuator.ArrayCommand` on ros topic `/uavcan_array_command`
* `std_msgs/Int32` to `uavcan.equipment.esc.RPMCommand` on ros topic `/ros_to_uavcan_bridge_node/rpm_command`


### UAVCAN to ROS
* `uavcan.equipment.ahrs.RawIMU` to `sensor_msgs/Imu` on ros topic `/uavcan_imu`
* `uavcan.equipment.gnss.Fix` to `sensor_msgs/NavSatFix` on ros topic `/uavcan_to_ros_bridge_node/gps_fix`
* `uavcan.equipment.power.BatteryInfo` to `sensor_msgs/BatteryState` on ros topic `/uavcan_to_ros_bridge_node/battery_state`
* `uavcan.equipment.ahrs.MagneticFieldStrength` to `sensor_msgs/MagneticField` on ros topic `/uavcan_to_ros_bridge_node/magnetic_field`
