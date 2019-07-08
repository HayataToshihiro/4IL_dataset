# 4IL_dataset

# Dependances
- [jsk_recognition](https://github.com/jsk-ros-pkg/jsk_recognition)
- [amsl_recog_msgs](https://github.com/Sadaku1993/amsl_recog_msgs)

## Published topics
- /image (sensor_msgs/Image)
	- image for imitation learning
- /cluster/bbox(jsk_recignition_msgs/BoundingBox)
	- all euclidian clustering bbox
- /cluster/bbox/pickup(jsk_recignition_msgs/BoundingBox)
	- euclidian clustering bbox like human size
- /cluster/points(sensor_msgs/PointCloud2)
- /cluster/points/pickup(sensor_msgs/PointCloud2)


## Subscribed topics
- /velodyne_obstacles(sensor_msgs/PointCloud2)
	- pointcloud removed ground
- /velodyne_clear(sensor_msgs/PointCloud2)
	- ground pointcloud

