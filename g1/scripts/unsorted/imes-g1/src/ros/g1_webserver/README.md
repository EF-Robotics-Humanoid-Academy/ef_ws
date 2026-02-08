# MYBOTSHOP Web Interface
![maintainer](https://img.shields.io/badge/Maintainer-Salman-blue)

## Operation

- Add password to configuration file and service names

- Run WebServer

```bash
ros2 launch g1_webserver webserver.launch.py
```

## Dependency

Core dependency

- ROS2
- `pip3 install Flask`

For Audio

- `pip3 install playsound`
- `pip3 install TTS`

For Audio CLI

- `sudo apt-get install espeak-ng`
    
## Configuration

- Update the ``robot_webserver.yaml``
- Update `.gltf` & name in `index.html` and `system.html`
- Update `video stream script`

## Parameters

This file contains the configuration parameters for the g1_webserver node. 
The parameters are used to configure the web server, including the IP address and port, 
as well as the robot's GPS coordinates, password, and other settings.

- `str` robot_rosbag_dir -- Directory where the rosbag files are stored
- `int` robot_rosbag_storage -- Split rosbag is file size in more that 1 Gb (in bytes)
- `int` robot_rosbag_duration -- Split rosbag if duration is more than 30 minutes (in seconds)
- `lis` robot_services -- List of services to be used by the webserver
- `str` robot_webserver -- Name of the webserver node
- `str` robot_map_topic -- Name of the map topic
- `str` robot_cmd_vel -- Name of the cmd_vel topic
- `str` robot_e_stop -- Name of the e_stop topic
- `flo` robot_gps_lat -- Latitude of the robot
- `flo` robot_gps_lon -- Longitude of the robot
- `str` robot_password -- Password for the robot
- `str` robot_gps_topic -- Name of the gps topic
- `str` robot_battery_topic -- Name of the battery topic

## Test Waypoints

```bash
ros2 topic pub -1 /mmp_0001/fixposition/navsatfix sensor_msgs/msg/NavSatFix "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'gps'
  },
  status: {
    status: 0,
    service: 1
  },
  latitude: 50.95358,
  longitude: 6.60168,
  altitude: 70.0,
  position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  position_covariance_type: 0
}"
```

## Remove

```bash
cd /opt/mybotshop/src/mybotshop/g1_webserver/g1_webserver/ && rm -rf librosnode.py libaudiogen.py  libwebserver.py ignore_python_binartor.bash ignore_compile.py
```