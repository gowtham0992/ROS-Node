# ROS-Node
Sidewalk Detector

Builded a ROS node called “sidewalk_detector” that consumes the replayed data of this rosbag and outputs the following topics:

(1) /sidewalk_detector/color/image_raw

(2) /sidewalk_detector/depth/points_in

(3) /sidewalk_detector/depth/points_out

Definitions: 

where (1) outputs the images from the topic "/camera/color/image_raw” with a visible highlight (e.g. a red mask) mapped over the set of pixels that are considered to be INSIDE the sidewalk

where (2) outputs a point cloud which contains the subset of points from the point cloud output by the topic "/camera/depth/points” that are considered to be INSIDE the sidewalk

where (3) outputs a point cloud which contains the subset of points from the point cloud output by the topic "/camera/depth/points” that are considered to be OUTSIDE the sidewalk
