# image_splitter

image splitter for ROS2

This package converts sensor_msgs/msg/Image into cv_image and saves it as jpeg image.

# Environment

- Ubuntu18.04
- ROS Eloquent

# How to Use

## Install

```bash
$ cd your_colcon_ws/src
$ git clone https://github.com/RenFukatsu/image_splitter.git
$ cd your_colcon_ws
$ colcon build --symlink-install --packages-select image_splitter
```

## Setting parameters

Open the launch/image_splitter-launch.py and edit lines 11 and 12.
Set your image topic and your save dir.

```py
remappings=[('image', 'your_image_topic')],
parameters=[{'hz':0.0, 'save_dir_path':'your_save_dir'}]
```

## Run

```bash
$ ros2 launch image_splitter image_splitter-launch.py
```

open another terminal

```bash
$ ros2 bag play your_bag_file
```