#!/usr/bin/env python2.7
import pyrealsense2 as rs
import rospy

from realsense_node import RealsenseNode


if __name__ == "__main__":

    async                = rospy.get_param('/realsense_streamer/async', default=False)
    enable_depth         = rospy.get_param('/realsense_streamer/enable-depth', default=True)
    enable_color         = rospy.get_param('/realsense_streamer/enable-color', default=True)
    enable_infra1        = rospy.get_param('/realsense_streamer/enable-infra1', default=True)
    enable_infra2        = rospy.get_param('/realsense_streamer/enable-infra2', default=True)
    enable_accel         = rospy.get_param('/realsense_streamer/enable-accel', default=True)
    enable_gyro          = rospy.get_param('/realsense_streamer/enable-gyro', default=True)
    enable_button        = rospy.get_param('/realsense_streamer/enable-button', default=False)
    enable_ir_emitter    = rospy.get_param('/realsense_streamer/enable-ir-emitter', default=True)
    enable_auto_exposure = rospy.get_param('/realsense_streamer/enable-auto-exposure', default=True)

    depth_width          = rospy.get_param('/realsense_streamer/depth-width', default=640)
    depth_height         = rospy.get_param('/realsense_streamer/depth-height', default=480)
    depth_fps            = rospy.get_param('/realsense_streamer/depth-fps', default=30)

    color_width          = rospy.get_param('/realsense_streamer/color-width', default=640)
    color_height         = rospy.get_param('/realsense_streamer/color-height', default=480)
    color_fps            = rospy.get_param('/realsense_streamer/color-fps', default=30)

    infra1_width         = rospy.get_param('/realsense_streamer/infra1-width', default=640)
    infra1_height        = rospy.get_param('/realsense_streamer/infra1-height', default=480)
    infra1_fps           = rospy.get_param('/realsense_streamer/infra1-fps', default=30)

    infra2_width         = rospy.get_param('/realsense_streamer/infra2-width', default=640)
    infra2_height        = rospy.get_param('/realsense_streamer/infra2-height', default=480)
    infra2_fps           = rospy.get_param('/realsense_streamer/infra2-fps', default=30)

    accel_fps            = rospy.get_param('/realsense_streamer/accel-fps', default=250)
    gyro_fps             = rospy.get_param('/realsense_streamer/gyro-fps', default=400)

    usb_port             = rospy.get_param('/realsense_streamer/usb-port', default='/dev/ttyACM0')
    config_path          = rospy.get_param('/realsense_streamer/config-path', default=None)

    # TODO: set auto exposure

    # TODO: make the type of each stream (e.g. rs.format.rgb8) changeable
    if config_path is None or config_path.lower() == 'none':
        config = rs.config()
        if enable_depth:
            config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, depth_fps)
        if enable_color:
            config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, color_fps) # the channel says bgr, but opencv/cv_bridge makes it rgb later
        if enable_infra1:
            config.enable_stream(rs.stream.infrared, 1, infra1_width, infra1_height, rs.format.y8, infra1_fps)
        if enable_infra2:
            config.enable_stream(rs.stream.infrared, 2, infra2_width, infra2_height, rs.format.y8, infra2_fps)
        if enable_accel:
            config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, accel_fps)
        if enable_gyro:
            config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, gyro_fps)
    else:
        config = config_path

    node = RealsenseNode()
    node.run(config=config, async=async, button_enabled=enable_button, usb_port=usb_port, enable_ir_emitter=enable_ir_emitter, enable_auto_exposure=enable_auto_exposure)
