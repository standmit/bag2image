FROM ros:noetic-ros-base

RUN bash -c "mkdir /tmp/share"

RUN bash -c " \
    mkdir -p ~/catkin_ws/src && \
    source /opt/ros/noetic/setup.bash && \
    cd ~/catkin_ws/src && \
    catkin_init_workspace \
"

RUN bash -c " \
    apt update && \
    apt install -y git \
"

RUN bash -c " \
    cd ~/catkin_ws/src && \
    git clone https://github.com/standmit/bag2image && \
    source /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths -y . && \
    cd ~/catkin_ws && \
    catkin_make \
"

ENTRYPOINT \
    bash -c " \
        source /opt/ros/noetic/setup.bash && \
        source ~/catkin_ws/devel/setup.bash && \
        bagname=\`ls -1 /tmp/share | head -n 1\` && \
        bagname=\${bagname%.*} && \
        cameradir=camera-\${bagname%.*} && \
        mkdir -p /tmp/share/\$cameradir && \
        rosrun bag2image extract_images /tmp/share/\$bagname.bag /android/tango1/camera_1/image/compressed /tmp/share/\$cameradir && \
        chmod -R go+rw /tmp/share/\$cameradir && \
        imufile=imu-\$bagname.tsv && \
        rosrun bag2image extract_imu /tmp/share/\$bagname.bag /android/tango1/imu > /tmp/share/\$imufile && \
        chmod go+rw /tmp/share/\$imufile && \
        gpsfile=gps-\$bagname.tsv && \
        rosrun bag2image extract_gps --nostamp /tmp/share/\$bagname.bag /android/tango1/fix > /tmp/share/\$gpsfile && \
        chmod go+rw /tmp/share/\$gpsfile && \
        magfile=compass-\$bagname.tsv && \
        rosrun bag2image extract_compass /tmp/share/\$bagname.bag /android/tango1/magnetic_field > /tmp/share/\$magfile && \
        chmod go+rw /tmp/share/\$magfile \
    "