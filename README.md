# ABYSS case study

## Prerequisites
Ubuntu >= 22.04
ROS2 Humble

## Build
- cd ~/abyss_case_study
- rosdep update
- rosdep install -i --from-path src --rosdistro humble -y
- colcon build --packages-select image_streams_merger

## Run
### Playback ROS2 Bag
- cd ~/abyss_case_study
- ros2 bag play ./src/image_streams_merger/resource/bag_files/case_study --loop --rate 0.5

### Run program
- cd ~/abyss_case_study
- source ./install/setup.bash
- ros2 run image_streams_merger image_streams_merger 

### View panoramic image

- $RQT
- Plugins > Visualization > Plot 
- Select /Panoramic image





