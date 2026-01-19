# A Multi-Vehicle Dataset with Camera, LiDAR, and Radar Sensors and Scanned 3D Models for Custom Auto-Annotation using RTK-GNSS

A dataset for autonomous driving, incorporating lidar, camera and radar sensors, seven simultaneous target vehicles with INS reference poses and 3D scans.

<img width="1525" height="311" alt="github_teaser" src="https://github.com/user-attachments/assets/5fe75c8d-45b1-495e-a4b1-a24eec8e05f0" />

**Authors:** Philipp Berthold*, Bianca Forkel*, and Mirko Maehlisch (* contributed equally)

**Abstract:**
_Datasets are a crucial element in the development of perception algorithms.
They relate sensor measurement data to annotated reference information
and allow for the deduction of sensor and object characteristics.
In autonomous driving, the reference data commonly consist of semantic image segmentation, point-wise associations, or bounding box annotations.
The dataset proposed in this work, however, aims to dig deeper into the evaluation of measurement principles and provides scanned 3D models of all vehicles together with a pose and continuous kinematics reference obtained by RTK-GNSS.
Combined, the state of the complete dynamic surrounding of the sensor vehicle is known for any point in time.
Subsequent reference formats can be easily computed in user-defined granularity.
This dataset involves single-object and multi-object recordings with seven target vehicles.
In particular, measurement effects such as occlusion, as well as reflections, can be evaluated, as the normals of the shape of the target vehicles are known.
We describe the dataset, discuss the technical background of its development, and briefly present exemplary evaluations._

**Citation:** If you use this dataset, please cite our paper such as
```
@InProceedings{7V-Scanario_2025,
  author    = {Philipp Berthold AND Bianca Forkel AND Mirko Maehlisch},
  booktitle = {Symposium Sensor Data Fusion (SDF)},
  title     = {{A Multi-Vehicle Dataset with Camera, LiDAR, and Radar Sensors and Scanned 3D Models for Custom Auto-Annotation using RTK-GNSS}},
  year      = {2025}
}
```

**Example videos:**

Multi-object scenario (rendered in rviz, without time-synchronization):

https://github.com/user-attachments/assets/28407284-4935-4290-8ff5-ac9e59e162c3


3D-Scan of a compact car (succeeded Artec Studio scan registration):

https://github.com/user-attachments/assets/83a12f25-8f94-4e5e-bfb7-644928d68034


Artifical lighting (may be used for physical data augmentation), shown in Blender:

https://github.com/user-attachments/assets/0f4eccfd-406b-43c4-95c3-4d0c8cf1b14f


Heatmap for radar detections (rendered in MATLAB as normalized 360°-view accumulation):

https://github.com/user-attachments/assets/210d9f38-a5f5-41e7-8b67-11e5d0c7c176


**Index:**
- [Vehicles](#vehicles): Description of the utilized vehicles
- [Sensors](#sensors): Description of the utilized sensors
- [Recordings](#recordings): Description of the recorded single-vehicle and multi-vehicle recordings
- [ROS Topics](#ros-bag-topics): Description of all ROS topics
- [Download](#download): Available material of this dataset (ROS bags, 3D scans, offline INS logs) and how to get all data
- [Example ROS code](#example-ros-code): Manual to run a rough data inspection in rviz

**License:** on request

## Vehicles

| Code | Type | Description |
| ------ | ----------- | -- |
| `i10`  | Hyundai i10 IA | compact class car, equipped with an INS setup mounted on the roof |
| `e87`  | BMW 1 series, E87 | compact class car, equipped with an INS setup mounted on the roof |
| `a6`  | Audi A6 | station wagon, equipped with an INS setup mounted on the roof |
| `tiguan`  | VW Tiguan Track & Field | sports utility vehicle, prototype vehicle with various sensors on the roof |
| `q8`  | Audi Q8 | sports utility vehicle, prototype vehicle with various sensors on the roof |
| `streetscooter`  | Streetscooter Work | small delivery transporter, prototype vehicle with various sensors on the roof |
| `crafter`  | VW e-Crafter | large transporter, prototype vehicle with various sensors on the roof |
 ---

## Sensors

Quantity | Sensor Type | Sensor Name | Rate (each) | Description |
| -- | ------ | ----------- | -- | -- |
| 4 | Camera  | `Basler acA2440-20gc` | 10 Hz | surround RGB cameras, mounted on the roof |
| 1 | LiDAR  | `Velodyne VLS128` | 10 Hz | mounted on the roof |
| 1 | Radar  | `Smartmicro UMRR-11` | ~18 Hz | far-range 77GHz Doppler radar, mounted on the front bumper |
| 5 | Radar  | `Smartmicro UMRR-96` | ~18 Hz | mid-range 77GHz Doppler radar, mounted on the front bumper and on all four vehicle corners |
| 1+n | INS | `Oxford OxTS RT3003` | 100 Hz | RTK-GNSS INS with two GNSS antennas and local RTK reference data (1 ego + n target vehicles) |
| 1 | Vehicle CAN data | `VW Touareg` (ego) | 20 Hz | vehicle series sensors like wheel speed and steering measurements |
 ---

## Recordings
### Single Vehicle Recordings

| Vehicle ID | Weather Conditions | Description |
| ------ | ----------- | -- |
| `i10` | cloudy | standard multi-perspective recording |
| `e87` | cloudy | standard multi-perspective recording |
| `a6`  | cloudy | standard multi-perspective recording |
| `a6`  | **rainy** | standard multi-perspective recording  |
| `tiguan`  | cloudy | standard multi-perspective recording  |
| `q8`  | cloudy | standard multi-perspective recording |
| `streetscooter` | cloudy | standard multi-perspective recording |
| `crafter` | cloudy | standard multi-perspective recording  |
 --- 

 ### Multi Vehicle Recordings

| ID | Location | Start Time | Duration | Description |
| ------ | ----------- | --|  -- | -- |
| `01_circles` | mehrzweck | 12-15-28 | 288s | The target vehicles circle in front of the stationary ego vehicle. |
| `02_figureEights` | mehrzweck | 12-21-30 | 266s | The target vehicles drive figure eights in front of the stationary ego vehicle. |
| `03_crossTraffic` | narrow crossing | 12-30-05 | 57s | The target vehicles pass by the stationary ego vehicle in a narrow crossing. Their appearance is sudden due to occlusion. |
| `04_crossTraffic` | narrow crossing | 12-31-11 | 26s | similar to `03_crossTraffic`
| `05_crossTraffic` | open crossing | 12-34-13 | 185s | The target vehicles pass by the stationary ego vehicle in an open crossing. The target vehicles simulate two-way traffic (causing occlusion). |
| `06_overtaking` | taxiway | 12-39-43 | 197s | The ego vehicle follows the target vehicles on a straight road. Repeatedly, the last target vehicle overtakes all other target vehicles. |
| `07_turning` | taxiway | 12-43-24 | 16s | The target vehicles turn on a two-way road. |
| `08_overtaking` | taxiway | 12-43-59 | 159s | All vehicles simulate one-way road traffic. The ego vehicle follows the target vehicles. |
| `09_highway` | taxiway | 12-47-21 | 146s | All vehicles simulate multi-lane highway traffic. The ego vehicle follows the target vehicles. |
| `10_highway` | taxiway+rundkurs | 12-50-51 | 260s | The vehicles first simulate multi-lane highway traffic, then exit the highway onto a winding road. The ego vehicle follows the target vehicles. |
| `11_circuit` | rundkurs | 12-57-19 | 158s | The ego vehicle overtakes all target vehicles on a bendy road on the left lane. |
| `12_circuit` | rundkurs+mehrzweck+taxiway | 13-01-18 | 258s | The vehicles drive on different road segments, performing some cut-ins. |
| `13_highway` | taxiway |13-07-12 |  137s | The vehicles simulate multi-lane highway traffic. |
| `14_countryRoad` | taxiway |13-12-34 |  63s | The vehicles simulate two-way traffic on a straight road. |
| `15_countryRoad` | taxiway | 13-14-27 |  87s | similar to `14_countryRoad` |
| `16_circles` | mehrzweck | 13-19-17 | 78s | The vehicles drive circles in a convoy. |
| `17_circles` | mehrzweck | 13-21-24 | 127s | The vehicles drive along two imaginary circles with different diameters around the same center. |
| `18_circles` | mehrzweck | 13-23-53 | 137s | The target vehicles drive along two imaginary circles in different directions around the stationary ego vehicle. |
| `19_circles` | mehrzweck | 13-26-46 | 142s | The target vehicles drive circles around the stationary ego vehicle. |
| `20_highway` | taxiway | 13-32-24 | 153s | The vehicles simulate multi-lane highway traffic. The ego vehicle switches lanes. |
| `21_ghostRider` | taxiway | 13-36-32 | 49s | The vehicles simulate two-way traffic: on the left and on the right lane of the ego vehicle, the target vehicles pass by. |
| `22_ghostRider` | taxiway | 13-38-55 | 46s | similar to `21_ghostRider` |
| `23_circuit` | taxiway+rundkurs+mehrzweck | 13-41-09 | 181s | The vehicles drive along winding road segments. The target vehicles drive on the right lane, the ego vehicle drives on the left lane (no overtaking). |
--- 

## ROS Bag Topics
The ROS bag contains the sensor data of the ego vehicle and all received WiFi data (INS data from target vehicles). We provide both raw Ethernet data and processed data for the local lidar, radar and vehicle (odometry) sensors and all INS units. This allows for utilizing pre-processed sensor data, and also for the utilization of custom data parsers / decoders. In latter case, the timestamp of the `ethernet_msgs/Packet` message header should be used for timestamp association. Concerning the camera sensors, raw images (+ camera info) are provided. Extrinsic sensor calibration is provided by the `tf_static` topic.

| Topic | Message Type | Description |
| ------ | ----------- | -- |
| `actuator/vehicle/feedback` | `vehicle_msgs/Feedback` | Ego vehicle data like wheel speed and steering information |
| `bus/oxts/eth_ncom/bus_to_host` | `ethernet_msgs/Packet` | Raw Ethernet data received from all INS units (OxTS NCOM UDP packets) |
| `bus/oxts/eth_rtcmv3/bus_to_host` | `ethernet_msgs/Packet` | Raw Ethernet data received from all INS units (forwarding of RTCMv3 UDP packets) |
| `bus/umrr/eth_measurements/bus_to_host` | `ethernet_msgs/Packet` | Raw Ethernet data received from the UMRR radar sensors (smartmicro transport protocol) |
| `bus/vls128/eth_scan/bus_to_host` | `ethernet_msgs/Packet` | Raw Ethernet data received from the VLS128 lidar sensor |
| `localization/egomotion/odom` | `nav_msgs/Odometry` | A simple 2D dead-reckoning egomotion that solely uses the series wheel speed sensors and a (2D) yaw rate gyroscope. Algorithms like SLAM using this egomotion can be compared to the INS reference solution without causing ground-truth leakage. |
| `sensor/camera/surround/<Camera ID>/camera_info` | `sensor_msgs/CameraInfo` | CameraInfo message for \<Camera ID\> |
| `sensor/camera/surround/<Camera ID>/image_raw` | `sensor_msgs/Image` | Raw image of \<Camera ID\>. Use the ROS image pipeline for rectified images. |
| `sensor/ins/oxts_rt3000/gps/fix` | `sensor_msgs/NavSatFix` | Processed NavSatFix message of the ego vehicle's OxTS INS unit |
| `sensor/ins/oxts_rt3000/gps/odom` | `nav_msgs/Odometry` | Processed Odometry message of the ego vehicle's OxTS INS unit. The pose information is given in UTM coordinates. |
| `sensor/ins/oxts_rt3000/imu/data` | `sensor_msgs/Imu` | Processed Imu message of the ego vehicle's OxTS INS unit |
| `sensor/ins/oxts_rt3000/objects` | `object_msgs/Objects` | INS pose and kinematics data from the ego vehicle and all target vehicles, attached with 3D dimensions. Emitted sequentially (for each INS) - no data synchronization. Only outputs INS data for vehicles in WiFi range. |
| `sensor/lidar/vls128_roof/velodyne_points` | `sensor_msgs/PointCloud2` | Processed PointCloud2 message of the VLS128 lidar sensor (without any egomotion compensation) |
| `sensor/radar/umrr/detections` | `radar_msgs/DetectionRecord` | Processed UMRR radar detections, collected over all UMRR sensors |
| `sensor/radar/umrr/pointclouds/<Radar ID>` | `sensor_msgs/PointCloud2` | Processed PointCloud2 message of the UMRR radar sensor \<Radar ID\> |
| `tf` | `tf2_msgs/TFMessage` | dynamic TF messages |
| `tf_static` | `tf2_msgs/TFMessage` | static TF messages, containing static mounting poses of the sensors (see frame_id) |
| `other/<Vehicle ID>/sensor/ins/oxts_rt3000/gps/fix` | `sensor_msgs/NavSatFix` | Processed NavSatFix message of the specific target vehicle's OxTS INS unit. Only available when target vehicle is in WiFi range. |
| `other/<Vehicle ID>/sensor/ins/oxts_rt3000/gps/odom` | `nav_msgs/Odometry` | Processed Odometry message of the specific target vehicle's OxTS INS unit. The pose information is given in UTM coordinates. Only available when target vehicle is in WiFi range. |
| `other/<Vehicle ID>/sensor/ins/oxts_rt3000/imu/data` | `sensor_msgs/Imu` | Processed Imu message of the specific target vehicle's OxTS INS unit. Only available when target vehicle is in WiFi range. |
--- 

### Dependencies
Most ROS message type formats are standard ROS messages, the remaining message definitions can be obtained from:
| Message Package | Where to get |
| ------ | ----------- |
| `vehicle_msgs` | https://github.com/UniBwTAS/vehicle_msgs |
| `ethernet_msgs` | https://github.com/UniBwTAS/ethernet_msgs (see https://github.com/UniBwTAS/ethernet_bridge for ROS Ethernet sockets to inspect data with manufacturer software during bag replay) |
| `object_msgs` | https://github.com/UniBwTAS/object_msgs |
| `radar_msgs` | https://github.com/UniBwTAS/tas_radar |
---

## Download
We are currently organizing the upload of the data (total: approx. 1 TB) to a publicly accessible platform. Until then, you are welcome to send us a hard drive with return postage. Please contact us for this.

### Recordings
All ROS bags are compressed and can be downloaded separately.
Simply download them to `/path/to/data/` and execute:

`cd /path/to/data && for f in *.tar.xz; do tar -xJf "$f"; done`

### 3D-Scans
The scans provide 3D contour and texture information for each vehicle used in this dataset. Each scan is aligned to the coordinate system (translation and rotation) of the **output data** of the INS of the vehicle. Background: while some INS units use the reference point of the IMU as the zero-point of its coordinate system, others use the centroid of the vehicle. The reference point is selected for best measurability. As the 3D scans are already aligned accordingly, you do not need to perform any object coordinate transformations.

#### Artec Studio
The 3D scans were obtained with the Artec Leo 3D hand scanner and using the Artec Studio for registration. Please contact us if you need the Artec Studio project files including the raw measurements (between 50GB and 200GB per vehicle).

#### Wavefront Objects
The processed 3D scans are provided as Wavefront Object (.obj-files with texture). You can use software like __[Blender](https://www.blender.org/)__ to convert the objects to a format of your choice.

#### Reduced Collada Files for rviz Visualization
We also provide a downsampled version of each vehicle as Collada file (.dae) for a straight-forward usage in rviz (using __[Markers](https://wiki.ros.org/rviz/DisplayTypes/Marker#Mesh_Resource_.28MESH_RESOURCE.3D10.29_.5B1.1.2B-.5D)__) for quick data inspection. Please note that loading all objects simultaneously still requires high computing resources, which might cause lags in rviz.

### INS (raw) data
The ROS bags contain the poses of the ego vehicle and all target vehicles (see ROS topic description). This data has been received via long-range WiFi during the recordings.
As a stable WiFi connection cannot be retained at high distances, the INS additionally logged their position solution (and some raw data) to a local storage. This data provides continuous positional and kinematic information. In addition, you can also use this data for post-processing (that usually yields a better estimation quality than the live solution).
A manual for the installation and operation of the processing software can be found here:
__[OxTS (INS) processing of RD files to NCOM or CSV](https://support.oxts.com/hc/en-us/articles/360010705400-NAVsolve-CMD-Command-Line-User-Guide)__.
The INS data is sorted by vehicle and time.

Please contact us if you need the raw data of the local GNSS reference station.

## Example ROS Code
This section outlines the installation of the ROS package that can be used for a rough data inspection (sensor data visualization + 3D models) of the dataset data in rviz.
Please note that **rviz does not perform time synchronization of the visualized data** (e.g., between all sensor measurements and the INS units that provide the poses of all 3D objects), which causes significant deviations of the visualized data. For any kind of data evaluation and processing, you need to use the provided timestamp in the ROS message headers for time synchronization. **The rviz rendering should only be used for rough data inspection.**
1. Add `object_msgs` (see [dependencies](#dependencies)) to your catkin workspace.
1. (optionally, to use raw data) Also add the other packages from [dependencies](#dependencies).
1. Add the contents of the folder `ros` of this repository to your catkin workspace.
1. Download the [Collada](#reduced-collada-files-for-rviz-visualization) files and unpack them to the `meshes` folder in the ROS package, i. e., to `scanario/meshes/`.
1. Compile your workspace using `catkin build`.
1. Start the launch file `roslaunch scanario inspect.launch bag:=<path_to_bag>`, where `<path_to_bag>` corresponds to the ROS Bag (path and filename) you would like to investigate, for example, `roslaunch scanario inspect.launch bag:=/mnt/data/multi-vehicle/09_highway/2024-12-05-12-47-21.bag`. The bag will be played, and an example rviz layout is started.
1. To view the 3D models, activate "mesh" (to be found in Tab `Displays` -> `Target Objects` -> `Marker` -> `Namespaces`). Please note that the initial loading of the 3D models will take up to a minute.
