import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy

import serial as Serial

from std_msgs.msg import Header, UInt32
from sensor_msgs.msg import Imu, Image

from pipeline import Pipeline


class RealsenseNode():
    def __init__(self):

        # TODO: this could be done more elegantly using a dict and the unique ID of each stream
        self.pub_color = rospy.Publisher('camera/color_image', Image, queue_size=10)
        self.pub_depth = rospy.Publisher('camera/depth_image', Image, queue_size=1)
        self.pub_depth_color_aligned = rospy.Publisher('camera/depth_color_aligned_image', Image, queue_size=1)
        self.pub_depth_infra_aligned = rospy.Publisher('camera/depth_infra_aligned_image', Image, queue_size=1)
        self.pub_infra1 = rospy.Publisher('camera/infrared1_image', Image, queue_size=1)
        self.pub_infra2 = rospy.Publisher('camera/infrared2_image', Image, queue_size=1)
        self.pub_accel = rospy.Publisher('camera/accel', Imu, queue_size=1)
        self.pub_gyro = rospy.Publisher('camera/gyro', Imu, queue_size=1)

        self.bridge = CvBridge()

        self.pipeline = Pipeline()
        #rs.log_to_file(rs.log_severity.debug, "filename.log")

        self.color_align = rs.align(rs.stream.color)
        self.infra_align = rs.align(rs.stream.infrared) # aligns to the left infrared camera

        rospy.init_node('realsense_node', anonymous=True)

    def callback(self, frame):
        try:
            # Generate a header for the frames
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = '/camera'

            #frame.get_timestamp() # timestamp (when frame was captured) of the frame, in milliseconds since the device was started (stored as float)
            #frame.get_frame_metadata(rs.frame_metadata_value.backend_timestamp) # when the frame was created(?) in computer time
            #frame.get_frame_metadata(rs.frame_metadata_value.time_of_arrival)

            # With callbacks, all synchronized streams (e.g. depth, color, infra1, and infra2) will arrive in a single frameset
            if frame.is_frameset():
                for f in frame.as_frameset(): # Though it should already be a frameset, we have to cast it to a frameset
                    self.handle_frame(frame=f, header=header)

                # publish the aligned frames
                color_aligned_depth = self.color_align.process(frame.as_frameset())
                image = np.asanyarray(color_aligned_depth.get_depth_frame().get_data())
                image = cv2.applyColorMap(cv2.convertScaleAbs(image, alpha=0.03), cv2.COLORMAP_JET)
                image_msg = self.bridge.cv2_to_imgmsg(image, 'rgb8')
                image_msg.header = header
                self.pub_depth_color_aligned.publish(image_msg)

                infra_aligned_depth = self.infra_align.process(frame.as_frameset())
                image = np.asanyarray(infra_aligned_depth.get_depth_frame().get_data())
                image = cv2.applyColorMap(cv2.convertScaleAbs(image, alpha=0.03), cv2.COLORMAP_JET)
                image_msg = self.bridge.cv2_to_imgmsg(image, 'rgb8')
                image_msg.header = header
                self.pub_depth_infra_aligned.publish(image_msg)

            else:
                self.handle_frame(frame=frame, header=header)

        except Exception as e:
            rospy.logwarn(e)

    def handle_frame(self, frame, header):

        # image frames
        try:
            if frame.get_profile().unique_id() == self.pipeline.get_active_profile().get_stream(rs.stream.depth).unique_id():
                image = np.asanyarray(frame.get_data())
                image = cv2.applyColorMap(cv2.convertScaleAbs(image, alpha=0.03), cv2.COLORMAP_JET)
                image_msg = self.bridge.cv2_to_imgmsg(image, 'rgb8')
                image_msg.header = header
                self.pub_depth.publish(image_msg)
        except RuntimeError as e:
            # We get a RuntimeError if the stream is not active and as for its unique ID anyway
            rospy.logwarn(e)

        try:
            if frame.get_profile().unique_id() == self.pipeline.get_active_profile().get_stream(rs.stream.infrared).unique_id():
                image = np.asanyarray(frame.get_data())
                image_msg = self.bridge.cv2_to_imgmsg(image, 'passthrough')
                self.pub_infra1.publish(image_msg)
        except RuntimeError as e:
            rospy.logwarn(e)

        try:
            if frame.get_profile().unique_id() == self.pipeline.get_active_profile().get_stream(rs.stream.infrared).unique_id()+1:
                image = np.asanyarray(frame.get_data())
                image_msg = self.bridge.cv2_to_imgmsg(image, 'passthrough')
                image_msg.header = header
                self.pub_infra2.publish(image_msg)
        except RuntimeError as e:
            rospy.logwarn(e)

        try:
            if frame.get_profile().unique_id() == self.pipeline.get_active_profile().get_stream(rs.stream.color).unique_id():
                image = np.asanyarray(frame.get_data())
                image_msg = self.bridge.cv2_to_imgmsg(image, 'passthrough')
                image_msg.header = header
                self.pub_color.publish(image_msg)
        except RuntimeError as e:
            rospy.logwarn(e)

        # imu frames
        try:
            if frame.get_profile().unique_id() == self.pipeline.get_active_profile().get_stream(rs.stream.accel).unique_id():
                accel = frame.as_motion_frame().get_motion_data()
                imu_msg = Imu()
                imu_msg.linear_acceleration.x = accel.x
                imu_msg.linear_acceleration.y = accel.y
                imu_msg.linear_acceleration.z = accel.z
                imu_msg.header = header
                self.pub_accel.publish(imu_msg)
        except RuntimeError as e:
            rospy.logwarn(e)

        try:
            if frame.get_profile().unique_id() == self.pipeline.get_active_profile().get_stream(rs.stream.gyro).unique_id():
                gyro = frame.as_motion_frame().get_motion_data()
                imu_msg = Imu()
                imu_msg.angular_velocity.x = gyro.x
                imu_msg.angular_velocity.y = gyro.y
                imu_msg.angular_velocity.z = gyro.z
                imu_msg.header = header
                self.pub_gyro.publish(imu_msg)
        except RuntimeError as e:
            rospy.logwarn(e)

    def button(self):
        ser_bytes = self.serial.read_all().decode("utf-8")

        # if we don't receive anything we continue in the state we are currently in NOTE: this creates a potential hangup
        if len(ser_bytes) < 1:
            return None

        response = ser_bytes[len(ser_bytes) - 1]  # we are only interested in the final (the latest) message
        if response == '0':
            return False
        elif response == '1':
            return True

    def load_config_from_file(self, config_path='config.json'):
        DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", "0B3A"]

        def find_device_that_supports_advanced_mode():
            ctx = rs.context()
            ds5_dev = rs.device()
            devices = ctx.query_devices();
            for dev in devices:
                if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
                    if dev.supports(rs.camera_info.name):
                        rospy.loginfo("Found device that supports advanced mode: {}".format(dev.get_info(rs.camera_info.name)))
                    return dev
            raise Exception("No device that supports advanced mode was found")

        dev = find_device_that_supports_advanced_mode()
        advnc_mode = rs.rs400_advanced_mode(dev)
        rospy.loginfo("Advanced mode is {}".format("enabled" if advnc_mode.is_enabled() else "disabled"))

        import json

        with open(config_path) as json_file:
            as_json_object = json.load(json_file)
            as_json_object = {k.encode('utf-8'): v.encode("utf-8") for k, v in as_json_object.items()}
            json_string = str(as_json_object).replace("'", '\"')
            advnc_mode.load_json(json_string)

            config = rs.config()
            config.enable_device(dev.get_info(rs.camera_info.serial_number))
            return config

    def run(self, config, async, button_enabled, usb_port, enable_ir_emitter=True, enable_auto_exposure=True):

        if isinstance(config, str):
            rospy.logwarn('Loading config file.. NOTE this is experimental and not tested so use at your own risk! You may have to disconnect and reconnect the camera to undo the configurations set.')
            config = self.load_config_from_file('config.json')

        assert config.can_resolve(self.pipeline), 'The configuration is not supported (verify the camera is connected and the image resolutions and framerates are supported with rs-enumerate-devices)'

        if button_enabled:
            self.serial = Serial.Serial(usb_port)
            rospy.sleep(1.5)  # the Arduino resets when we connect to it, so we give it some time to complete.

        # this if/else is abysmal and can probably be made a lot cleaner
        if async:
            rate = rospy.Rate(10)  # 10hz
            while not rospy.is_shutdown():
                if button_enabled:
                    button_state = self.button()
                    if button_state:
                        self.pipeline.start(config, self.callback, enable_ir_emitter=enable_ir_emitter, enable_auto_exposure=enable_auto_exposure)
                    elif not button_state:
                        try:
                            # Using pipeline.stop() doesn't work when we use a callback (dunno why), so we stop each sensor individually though its not very clean as some sensors throw a RuntimeError (because they can't be stopped?)
                            for sensor in self.pipeline.get_active_profile().get_device().sensors:
                                sensor.stop()
                                sensor.close()
                        except RuntimeError:
                            pass
                        self.pipeline.stop()
                    else:
                        pass
                else:
                    self.pipeline.start(config, self.callback, enable_ir_emitter=enable_ir_emitter, enable_auto_exposure=enable_auto_exposure)

                rate.sleep()

        else:
            while not rospy.is_shutdown():
                if button_enabled:
                    button_state = self.button()
                    if button_state is True:
                        self.pipeline.start(config, enable_ir_emitter=enable_ir_emitter, enable_auto_exposure=enable_auto_exposure)
                        frames = self.pipeline.wait_for_frames()
                        self.callback(frame=frames) # We can call the callback function manually which can handle the framesets we get from pipline.wait_for_frames()
                    elif button_state is False:
                        self.pipeline.stop()
                    else:
                        pass
                else:
                    self.pipeline.start(config, enable_ir_emitter=enable_ir_emitter, enable_auto_exposure=enable_auto_exposure)
                    frames = self.pipeline.wait_for_frames()
                    self.callback(frame=frames) # We can call the callback function manually which can handle the framesets we get from pipline.wait_for_frames()
