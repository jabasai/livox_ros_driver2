# Livox ROS Driver 2

Livox ROS Driver 2 is the 2nd-generation driver package used to connect LiDAR products produced by Livox.

> **Note:** This branch targets **ROS2 only** (Humble or later recommended). ROS1 is not supported. The build system defaults to ROS2 Humble when invoked without arguments via `colcon`.

  **Note :**

  As a debugging tool, Livox ROS Driver is not recommended for mass production but limited to test scenarios. You should optimize the code based on the original source to meet your various needs.

## 1. Preparation

### 1.1 OS requirements

  * Ubuntu 20.04 for ROS2 Foxy;
  * Ubuntu 22.04 for ROS2 Humble (recommended);

  **Tips:**

  Colcon is the build tool used for ROS2.

  How to install colcon: [Colcon installation instructions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

### 1.2 Install ROS2

For ROS2 Foxy installation, please refer to:
[ROS2 Foxy installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

For ROS2 Humble installation, please refer to:
[ROS2 Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Desktop-Full installation is recommended.

## 2. Build & Run Livox ROS Driver 2

### 2.1 Clone Livox ROS Driver 2 source code:

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

  **Note :**

  Be sure to clone the source code in a '[work_space]/src/' folder (as shown above), otherwise compilation errors will occur due to the compilation tool restriction.

### 2.2 Build & install the Livox-SDK2

  **Note :**

  Please follow the guidance of installation in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)

### 2.3 Build the Livox ROS Driver 2:

#### For ROS2 Foxy:
```shell
source /opt/ros/foxy/setup.sh
colcon build
```

#### For ROS2 Humble (recommended):
```shell
source /opt/ros/humble/setup.sh
colcon build
```

The build defaults to ROS2 Humble when the `ROS_DISTRO` and `ROS_VERSION` environment variables are set (as they are after sourcing a ROS2 setup script). No additional arguments are required.

### 2.4 Run Livox ROS Driver 2:

```shell
source ../../install/setup.sh
ros2 launch livox_ros_driver2 [launch file]
```

in which,  

* **[launch file]** : is the ROS2 launch file you want to use; the 'launch_ROS2' folder contains several launch samples for your reference.

A rviz launch example for HAP LiDAR would be:

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. Launch file and livox_ros_driver2 internal parameter configuration instructions

### 3.1 Launch file configuration instructions

Launch files are in the `launch/` directory. Different launch files have different configuration parameter values and are used in different scenarios:

| Launch file | Description |
| --- | --- |
| `rviz_HAP_launch.py` | Connect to HAP LiDAR device, publish PointCloud2 data, autoload RViz |
| `msg_HAP_launch.py` | Connect to HAP LiDAR device, publish Livox customized pointcloud data |
| `rviz_MID360_launch.py` | Connect to MID360 LiDAR device, publish PointCloud2 data, autoload RViz |
| `msg_MID360_launch.py` | Connect to MID360 LiDAR device, publish Livox customized pointcloud data |
| `rviz_mixed.py` | Connect to HAP and MID360 LiDAR devices, publish PointCloud2 data, autoload RViz |
| `mid360_launch.py` | **Configurable** single MID360 driver — IP, ports, namespace and host IP fully configurable via launch arguments or environment variables; generates config file automatically (see §3.3) |
| `dual_mid360_launch.py` | Launches two independent MID360 drivers (front + back) each in their own namespace, with non-conflicting port ranges; fully configurable via environment variables (see §3.3) |

### 3.2 Livox ROS Driver 2 internal main parameter configuration instructions

All internal parameters of Livox_ros_driver2 are in the launch file. Below are detailed descriptions of the three commonly used parameters :

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | Set the frequency of point cloud publish <br>Floating-point data type, recommended values 5.0, 10.0, 20.0, 50.0, etc. The maximum publish frequency is 100.0 Hz.| 10.0    |
| multi_topic  | If the LiDAR device has an independent topic to publish pointcloud data<br>0 -- All LiDAR devices use the same topic to publish pointcloud data<br>1 -- Each LiDAR device has its own topic to publish point cloud data | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTLT) pointcloud format<br>1 -- Livox customized pointcloud format<br>2 -- Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library (**not supported in ROS2**) | 0       |

  **Note :**

  Other parameters not mentioned in this table are not suggested to be changed unless fully understood.

&ensp;&ensp;&ensp;&ensp;***Livox_ros_driver2 pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTLT) point cloud format, as follows :

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```
  **Note :**

  The number of points in the frame may be different, but each point provides a timestamp.

2. Livox customized data package format, as follows :

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

&ensp;&ensp;&ensp;&ensp;Customized Point Cloud (CustomPoint) format in the above customized data package :

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # livox tag
uint8   line            # laser number in lidar
```

3. The standard pointcloud2 (pcl :: PointXYZI) format in the PCL library (**not supported in ROS2**):

&ensp;&ensp;&ensp;&ensp;Please refer to the pcl :: PointXYZI data structure in the point_types.hpp file of the PCL library.

---

### 3.3 Configurable MID360 launch files

`mid360_launch.py` and `dual_mid360_launch.py` replace the old static-JSON workflow for MID360 sensors. Instead of editing a JSON config file by hand, the LiDAR IP address, host IP, and UDP ports are resolved at launch time from **launch arguments** or **environment variables**, and a temporary config file is generated automatically in `/tmp/`. The temp file is deleted automatically when the launch session ends.

#### 3.3.1 `mid360_launch.py` — single MID360

Launches one MID360 driver node. All connectivity parameters can be supplied via launch arguments or environment variables. Environment variables are only consulted when the corresponding launch argument is left at its default empty value, so launch arguments always take full precedence.

**Parameter resolution order:** launch argument → environment variable → built-in default

##### Launch arguments

| Argument | Env var (no namespace) | Env var (namespace=`NS`) | Default | Description |
| --- | --- | --- | --- | --- |
| `namespace` | — | — | *(empty)* | ROS node/topic namespace **and** env-var prefix (uppercased). Empty = no namespace. |
| `livox_ip` | `LIVOX_IP` | `NS_LIVOX_IP` | `192.168.1.12` | IP address of the MID360 lidar. |
| `livox_host_ip` | `LIVOX_HOST_IP` | `NS_LIVOX_HOST_IP` | *(auto-detected)* | Host NIC IP the lidar sends data to. When empty, auto-detected via kernel routing table. |
| `livox_port_base` | `LIVOX_PORT_BASE` | `NS_LIVOX_PORT_BASE` | `56101` | First UDP host-side listening port. Ports `base … base+4` are used (cmd, push, point, imu, log). |
| `frame_id` | — | — | `<namespace>` or `livox_frame` | TF frame ID. Defaults to the namespace value when set, otherwise `livox_frame`. |
| `node_name` | — | — | `livox_lidar` | ROS node name. |
| `xfer_format` | — | — | `0` | Point cloud format (0=PointXYZRTLT, 1=custom). |
| `multi_topic` | — | — | `1` | Per-LiDAR topic (1) or shared topic (0). |
| `publish_freq` | — | — | `10.0` | Publish frequency in Hz. |

##### Port layout

Given `livox_port_base=N`, the five host UDP ports are assigned as:

| Channel | Host port |
| --- | --- |
| cmd_data | N |
| push_msg | N+1 |
| point_data | N+2 |
| imu_data | N+3 |
| log_data | N+4 |

The lidar-side ports (56100, 56200, 56300, 56400, 56500) are fixed by firmware and are always written verbatim into the generated config.

##### Examples

```shell
# Minimal — all defaults
ros2 launch livox_ros_driver2 mid360_launch.py

# Explicit launch arguments
ros2 launch livox_ros_driver2 mid360_launch.py \
  livox_ip:=192.168.1.167 livox_port_base:=56101

# Via environment variables (no namespace prefix)
LIVOX_IP=192.168.1.167 ros2 launch livox_ros_driver2 mid360_launch.py

# With namespace — node appears as /front_lidar/livox_lidar
# and env vars are read with FRONT_LIDAR_ prefix
FRONT_LIDAR_LIVOX_IP=192.168.1.167 \
  ros2 launch livox_ros_driver2 mid360_launch.py namespace:=front_lidar

# Fully explicit
ros2 launch livox_ros_driver2 mid360_launch.py \
  namespace:=front_lidar \
  livox_ip:=192.168.1.167 \
  livox_host_ip:=192.168.1.10 \
  livox_port_base:=56101
```

---

#### 3.3.2 `dual_mid360_launch.py` — two MID360 sensors

Launches two independent `mid360_launch.py` instances — one for a **front** lidar and one for a **back** lidar — each in their own ROS namespace with non-overlapping UDP port ranges.

**Default port layout:**

| Lidar | Namespace | Ports |
| --- | --- | --- |
| Front | `front_lidar` | 56101–56105 |
| Back | `back_lidar` | 56111–56115 |

##### Launch arguments

| Argument | Env var | Default | Description |
| --- | --- | --- | --- |
| `front_namespace` | — | `front_lidar` | Namespace for the front lidar. |
| `back_namespace` | — | `back_lidar` | Namespace for the back lidar. |
| `front_livox_ip` | `FRONT_LIDAR_LIVOX_IP` | `192.168.1.167` | IP of the front MID360. |
| `back_livox_ip` | `BACK_LIDAR_LIVOX_IP` | `192.168.1.184` | IP of the back MID360. |
| `front_port_base` | `FRONT_LIDAR_LIVOX_PORT_BASE` | `56101` | First UDP host port for the front lidar. |
| `back_port_base` | `BACK_LIDAR_LIVOX_PORT_BASE` | `56111` | First UDP host port for the back lidar. |

Host IP (`FRONT_LIDAR_LIVOX_HOST_IP` / `BACK_LIDAR_LIVOX_HOST_IP`) and other per-lidar parameters can still be set via the namespace-prefixed env vars defined in §3.3.1.

##### Examples

```shell
# Both IPs via environment variables
FRONT_LIDAR_LIVOX_IP=192.168.1.167 \
BACK_LIDAR_LIVOX_IP=192.168.1.184 \
  ros2 launch livox_ros_driver2 dual_mid360_launch.py

# Override one IP via launch argument
ros2 launch livox_ros_driver2 dual_mid360_launch.py \
  front_livox_ip:=192.168.1.167 back_livox_ip:=192.168.1.184

# Custom port bases (e.g. if 56101 is already in use)
ros2 launch livox_ros_driver2 dual_mid360_launch.py \
  front_port_base:=56201 back_port_base:=56211
```

Each lidar node is accessible under its own namespace:
- `/front_lidar/livox_lidar` — TF frame `front_lidar`
- `/back_lidar/livox_lidar` — TF frame `back_lidar`

---

## 4. LiDAR config

LiDAR Configurations (such as ip, port, data type... etc.) can be set via a json-style config file. Config files for single HAP, Mid360 and mixed-LiDARs are in the "config" folder. The parameter naming *'user_config_path'* in launch files indicates such json file path.

1. Follow is a configuration example for HAP LiDAR (located in config/HAP_config.json):

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "device_type" : "HAP",
    "lidar_ipaddr": "",
    "lidar_net_info" : {
      "cmd_data_port": 56000,  # command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip (it can be revised)
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the LiDAR you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

The parameter attributes in the above json file are described in the following table :

**LiDAR configuration parameter**
| Parameter                  | Type    | Description                                                  | Default         |
| :------------------------- | ------- | ------------------------------------------------------------ | --------------- |
| ip             | String  | Ip of the LiDAR you want to config | 192.168.1.100 |
| pcl_data_type             | Int | Choose the resolution of the point cloud data to send<br>1 -- Cartesian coordinate data (32 bits)<br>2 -- Cartesian coordinate data (16 bits) <br>3 --Spherical coordinate data| 1           |
| pattern_mode                | Int     | Space scan pattern<br>0 -- non-repeating scanning pattern mode<br>1 -- repeating scanning pattern mode <br>2 -- repeating scanning pattern mode (low scanning rate) | 0               |
| blind_spot_set (Only for HAP LiDAR)                 | Int     | Set blind spot<br>Range from 50 cm to 200 cm               | 50               |
| extrinsic_parameter |      | Set extrinsic parameter<br> The data types of "roll" "picth" "yaw" are float <br>  The data types of "x" "y" "z" are int<br>               |

For more infomation about the HAP config, please refer to:
[HAP Config File Description](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description)

2. When connecting multiple LiDARs, add objects corresponding to different LiDARs to the "lidar_configs" array. Examples of mixed-LiDARs config file contents are as follows :

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "lidar_net_info" : {  # HAP ports, please don't revise these values
      "cmd_data_port": 56000,  # HAP command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "MID360": {
    "lidar_net_info" : {  # Mid360 ports, please don't revise these values
      "cmd_data_port": 56100,  # Mid360 command port
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",  # host ip
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the HAP you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    {
      "ip" : "192.168.1.12",  # ip of the Mid360 you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```
3. when multiple nics on the host connect to multiple LiDARs, you need to add objects corresponding to different LiDARs to the lidar_configs array. Run different launch files separately. The following is an example of a mixed LiDAR configuration for reference — note that the launch file examples below use ROS1 XML syntax; for ROS2, equivalent Python launch files should be used instead:

**MID360_config1:**
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.1.100"], # Lidar ip
                "host_ip": "192.168.1.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.1.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```
**MID360_config2:**
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.2.100"], # Lidar ip
                "host_ip": "192.168.2.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.2.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```
**Launch1:**
```
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end--> 

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config1.json"/> # Mid360 MID360_config1 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher1" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>

</launch>
```
**Launch2:**
```
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end--> 

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config2.json"/> # Mid360 MID360_config2 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher2" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>

</launch>

```

## 5. Utility Tools

### 5.1 livox_set_ip — Change a LiDAR's IP address

`livox_set_ip` is a standalone command-line tool that uses Livox-SDK2 to assign a new static IP address to any connected Livox LiDAR (e.g. MID-360, HAP). It is useful for initial device setup or when you need to move a LiDAR to a different subnet.

**Usage:**

```shell
ros2 run livox_ros_driver2 livox_set_ip <current_lidar_ip> <new_ip> <netmask> <gateway> [host_ip]
```

| Argument | Description |
| --- | --- |
| `current_lidar_ip` | Current IP of the LiDAR to configure |
| `new_ip` | New static IP to assign to the LiDAR |
| `netmask` | Subnet mask (e.g. `255.255.255.0`) |
| `gateway` | Gateway address (e.g. `192.168.1.1`) |
| `host_ip` | *(optional)* IP of the host NIC connected to the LiDAR; omit to let the SDK auto-detect |

**Examples:**

```shell
# Change lidar IP; host NIC is explicitly specified
ros2 run livox_ros_driver2 livox_set_ip 192.168.1.1xx 192.168.1.167 255.255.255.0 192.168.1.1 192.168.1.10

# Host IP auto-detected by the SDK
ros2 run livox_ros_driver2 livox_set_ip 192.168.1.167 192.168.1.184 255.255.255.0 192.168.1.1
```

> **IMPORTANT:** Power-cycle the LiDAR after the tool reports success for the new IP address to take effect.

### 5.2 livox_scan — Discover LiDAR devices on the network

`livox_scan` listens for Livox device announcements for a configurable period and prints a summary table of every device found, including its IP address, device type, and serial number.

**Usage:**

```shell
ros2 run livox_ros_driver2 livox_scan [OPTIONS] [host_ip [timeout_seconds]]
```

**Options:**

| Option | Description |
| --- | --- |
| `-h`, `--help` | Show help message and exit |

**Arguments:**

| Argument | Description |
| --- | --- |
| `host_ip` | *(optional)* IP of the host NIC facing the LiDARs; omit to let the SDK auto-detect |
| `timeout_seconds` | *(optional)* How long to scan in seconds (default: `10`) |

**Examples:**

```shell
# Show help
ros2 run livox_ros_driver2 livox_scan --help

# Scan all NICs for 10 seconds (default)
ros2 run livox_ros_driver2 livox_scan

# Scan from a specific NIC
ros2 run livox_ros_driver2 livox_scan 192.168.1.10

# Scan from a specific NIC with a 5-second timeout
ros2 run livox_ros_driver2 livox_scan 192.168.1.10 5
```

**Example output:**

```
=== Livox LiDAR scanner ===
  Host NIC : (auto-detect)
  Timeout  : 10 s

[info] Scanning for 10 second(s) ...

[found] 192.168.1.167  type=Mid-360  sn=0TFDH7600601234
[found] 192.168.1.184  type=Mid-360  sn=0TFDH7600605678

[result] Found 2 device(s):

+------------------+------------------+--------------------+
| IP Address       | Device Type      | Serial Number      |
+------------------+------------------+--------------------+
| 192.168.1.167    | Mid-360          | 0TFDH7600601234    |
| 192.168.1.184    | Mid-360          | 0TFDH7600605678    |
+------------------+------------------+--------------------+
```

---

## 6. Supported LiDAR list

* HAP
* Mid360
* (more types are comming soon...)

## 7. FAQ

### 7.1 launch with "livox_lidar_rviz_HAP.launch" but no point cloud display on the grid?

Please check the "Global Options - Fixed Frame" field in the RViz "Display" pannel. Set the field value to "livox_frame" and check the "PointCloud2" option in the pannel.

### 7.2 launch with command "ros2 launch livox_lidar_rviz_HAP_launch.py" but cannot open shared object file "liblivox_sdk_shared.so" ?

Please add '/usr/local/lib' to the env LD_LIBRARY_PATH.

* If you want to add to current terminal:

  ```shell
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  ```

* If you want to add to current user:

  ```shell
  vim ~/.bashrc
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  source ~/.bashrc
  ```
