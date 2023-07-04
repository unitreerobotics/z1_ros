## z1_controller 

Current version temporarily dedicated to ROS driver and communicates with SDK via UDP structs.

## Parameters

+ **udp/mcu_ip**: The robot arm IP address
+ **udp/port_to_mcu**: The port specified when communicating with the robot arm, which needs to be changed when controlling multiple arms.
+ **udp/port_to_sdk**: The port specified when communicating with the SDK, which needs to be changed when controlling multiple arms
+ **collision/open**: Whether to start collision detection.
+ **collision/limitT**: Collision detection limited torque.
+ **cmdCache**: Whether to allow command caching
  If turned on, when z1_controller does not receive a message from the SDK in a single communication, it will execute shift_command to read commands from the cache (Specified by the user)

### State

+ For the time being only examples of  `joint control` and `lowcmd` control are available, other interfaces will follow.
+ The joint gain coefficients in lowcmd (25.6, 0.0128) thar were in [z1_controller_V2](https://dev-z1.unitree.com/use/introduction.html) have been removed and are now standard joint gains.
