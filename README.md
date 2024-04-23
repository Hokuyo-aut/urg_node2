# Overview
This package provides a ROS2 driver node for HOKUYO 2D LiDAR(SOKUIKI Sensor).

- Feature
  - Supports Ethernet and USB connections
  - Scanning data output
  - Multi-Echo Scanning data output (use `laser_proc` topic)
  - Output hardware diagnostic information (use `diagnostic_updater` topic)
  - Component Mounting
  - [Life cycle control](http://design.ros2.org/articles/node_lifecycle.html)support

The interfaces and some of the processes are based on [urg_node package](http://wiki.ros.org/urg_node), which is often used in ROS1.

[urg_library ver.1.2.5](https://github.com/UrgNetworks/urg_library/tree/ver.1.2.5) is used for communication with LiDAR.

## Life cycle control
The operation in each state of the life cycle is as follows
- Unconfigured  
  startup state
- Inactive  
  LiDAR connection state（not delivered scanning data and diagnostic information）
- Active  
  LiDAR data delivery state（delivered scanning data and diagnostic information）
- Finalized  
  end state

# Supported models
Hokuyo's SCIP 2.2-compliant LiDAR
Tested models: `UTM-30LX-EW`, `UST-10LX`, `UTM-30LX`, `URG-04LX-UG01`, `UAM-05LP`, `UAM-05LPA`

Tested Environments: `foxy`, `galactic`, `humble`

# License
`Apache License 2.0`
The urg_libray C API is licensed under the `Simplified BSD License`.

# Publish
- /scan (sensor_msgs::msg::LaserScan)  
  LiDAR scanning data (output when parameter `publish_multiecho`=false)
- /echoes (sensor_msgs::msg::MultiEchoLaserScan)  
  LiDAR multi-echo scanning data (output when parameter `publish_multiecho`=true)
- /first (sensor_msgs::msg::LaserScan)  
  Nearest scanning data (output when parameter `publish_multiecho`=true)
- /last (sensor_msgs::msg::LaserScan)  
  Farthest scanning data (output when parameter `publish_multiecho`=true)
- /most_intense (sensor_msgs::msg::LaserScan)  
  Most intense scanning data (output when parameter `publish_multiecho`=true)
- /diagnostics (diagnostics_msgs::msg::DiagnosticArray)  
  Diagnostic information

# Parameters
- ip_address (string, default: "")  
  IP address for Ethernet connection（Specify in the format "XX.XX.XX.XX.XX"）  
  ※If the specified string is empty, a serial connection is selected
- ip_port (int, default: 10940)  
  Port number for Ethernet connection
- serial_port (string, default: "/dev/ttyACM0")  
  Serial device path to connect to
- serial_baud (int, default: 115200)  
  Serial baud rate
- frame_id (string, default: "laser")  
  Frame_id of scanning data
  The parameter `frame_id` is set in the message header "frame_id" of the scan data.
- calibrate_time (bool, default: false)  
  Adjustment mode flags
  If this flag is true, the discrepancy between the LiDAR time and the system time is measured at the start of the scan and added to the timestamp of the scan data as latency.
- synchronize_time (bool, default: false)  
  Synchronous mode flags
  If this flag is true, the system time, which is the reference for the timestamp of the scan data, is dynamically corrected using the discrepancy from the LiDAR time.
- publish_intensity (bool, default: false)  
  Intensity output mode flag
  If this flag is true, the intensity data of the scan data is output; if false, the intensity data is output empty.  
  If LiDAR does not support intensity output, it works the same as the false setting.
- publish_multiecho (bool, default: false)  
  Multi-echo mode flag
  If this flag is true, multi-echo scan data (/echoes, /first, /last, /most_intense) is output; if false, scan data (/scan) is output.  
  If LiDAR does not support multi-echo, it works the same as the false setting.
- error_limit (int, default: 4 [count])  
  Number of errors to perform reconnection
  Reconnects the connection with LiDAR when the number of errors that occurred during data acquisition becomes larger than error_limit.    
- error_reset_period (double, default: 5.0 [sec])  
  Error reset cycle 
  Periodically resets the number of errors that occurred during data acquisition.  
  ※To prevent reconnection in case of sporadic errors
- diagnostics_tolerance (double, default: 0.05)  
  Allowable percentage of diagnostic information on frequency of scan data delivery relative to target delivery frequency  
  ※For example, if diagnostics_tolerance is 0.05, the normal range is 105% to 95% of the target delivery frequency.
- diagnostics_window_time (double, default: 5.0 [sec])  
  Period to measure the frequency of output delivery of diagnostic information on the frequency of scan data delivery.
- time_offset (double, default: 0.0 [sec])  
  User latency to be added to timestamp of scan data
- angle_min (double, default：-pi [rad], range：-pi～pi)  
  Minimum angle of scan data output range
- angle_max (double, default：pi [rad], range：-pi～pi)  
  Maximum angle of scan data output range
- skip (int, default: 0 [count], range: 0～9)  
  Output thinning setting for scanned data  
  The frequency of scanned data delivery is multiplied by 1/(skip+1).  
- cluster (int, default: 1 [count], range: 1～99)  
  Scan data grouping settings 
  The number of data in the scan data is multiplied by 1/cluster.

# How to build

1. Obtaining Source Code

```
$ cd <ROS2_workspace>/src
$ git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git
```

2. Installing Related Packages

```
$ rosdep update
$ rosdep install -i --from-paths urg_node2
```

3. Build

```
$ cd <ROS2_workspace>
$ colcon build --symlink-install
```

4. Testing (optional)

Since the test is executed for `UTM-30LX-EW`, please connect `UTM-30LX-EW` and execute it with the power ON.

```
$ cd <ROS2_workspace>
$ colcon test
```

# Examples of Use

## launch

1. Connect LiDAR  
   Connect via Ethernet or USB. 
1. Set the connection destination (parameters)   
   Edit `config/params_ether.yaml` (for Ethernet connections)   
   ※If you use USB connection, edit `config/params_serial.yaml' and change the parameter file specification part of `launch/urg_node2.launch.py` to params_serial.yaml.
1. Node startup
   Execute the following command to automatically transition to the Active state and begin distribution of scan data.

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py
   ```

   If you do not want to automatically transition to the Active state, execute the following command. (After startup, it will enter the Unconfigured state.)

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py auto_start:=false
   ```

   In `urg_node2.launch.py`, urg_node2 is launched as a standalone lifecycle node (not as a component). This is because there is no lifecycle control interface in launch when launched as a component and lifecycle node (not implemented in ROS2 at this time).

## Parameter Change

To change the parameters, perform the following steps.

1. Node Termination
   \<Ctrl-C\> to terminate the node.
1. Reset parameters  
   Edit the parameter file you are using (e.g. config/params_ether.yaml).
1. Restarting a node  
   Start the node with the following command

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py
   ```

## Reset in case of abnormality

If you encounter abnormalities, such as no scan data being delivered, please follow the steps below.

1. Node Termination
   \<Ctrl-C\> to terminate the node.
1. Connection Confirmation  
   Check if the LiDAR power supply and connections are normal.
1. Restarting a node  
   Start the node with the following command

   ```
   $ ros2 launch urg_node2 urg_node2.launch.py
   ```

# Restrictions

- in Galactic, Inactive->Active state transitions fail for the second and subsequent times.

When diagnostic_updater is generated, declare_parameter is always executed. The state transition cannot take place because of the failure of diagnostic_updater generation.  
This limitation will be resolved when the following pull request is reflected. 
  https://github.com/ros/diagnostics/pull/227

