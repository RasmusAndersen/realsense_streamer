import pyrealsense2 as rs
import rospy


class Pipeline(rs.pipeline):
    def __init__(self):
        self.is_active = False
        super(Pipeline, self).__init__()

    def start(self, config, callback=None, enable_ir_emitter=True, enable_auto_exposure=True):
        if not self.is_active:
            rospy.loginfo('Starting streams..')
            self.is_active = True
            if callback is None:
                profile = super(Pipeline, self).start(config)
                profile.get_device().first_depth_sensor().set_option(rs.option.emitter_enabled, enable_ir_emitter)
                profile.get_device().query_sensors()[1].set_option(rs.option.enable_auto_exposure, enable_auto_exposure)
                return profile
            else:
                profile = super(Pipeline, self).start(config, callback)
                profile.get_device().first_depth_sensor().set_option(rs.option.emitter_enabled, enable_ir_emitter)
                return profile

    def stop(self):
        if self.is_active:
            rospy.loginfo('Stopping streams')
            self.is_active = False
            return super(Pipeline, self).stop()
