# Zed Odom to base_link odom

This package provides a solution to transform the odometry data from the ZED camera's `camera_link` frame to the `base_link` frame. It addresses the issue where the ZED node publishes odometry with `camera_link` as the child frame, which is not suitable for applications requiring `base_link` as the reference frame. The odometry data needs to be converted to the odometry of 'base_link', like mentioned in [here](https://github.com/stereolabs/zed-ros-wrapper/issues/361)


# dependency
scipy
```
pip3 install scipy
```