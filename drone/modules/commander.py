import logging
import threading
import rospy
import modules.animation as animation
import modules.flight as flight
import modules.led as led
import modules.network as network
from modules.config import config

logger = logging.getLogger(__name__)

ANIMATION_PATH = "animation.csv"

class TaskManager:

    def __init__(self, animation_path=ANIMATION_PATH):
        self.animation = animation.Animation(filepath=animation_path, config=config)
        self.current_task = None
        self.interrupter = threading.Event()

        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()

    def do_action(self, action_name, **kwargs):
        self.interrupter.set()
        rospy.sleep(0.3)
        self.interrupter.clear()
        self.current_task = (action_name, kwargs)

    def _worker(self):
        while not rospy.is_shutdown():
            if self.current_task:
                action, kwargs = self.current_task
                self.current_task = None
                kwargs["interrupter"] = self.interrupter

                try:
                    self._dispatch(action, kwargs)
                except Exception as e:
                    logger.error("Action '%s' failed: %s", action, e)

            rospy.sleep(0.1)

    def _dispatch(self, action, kwargs):
        if action == "takeoff":
            flight.takeoff(**kwargs)
        elif action == "navto":
            flight.navto(**kwargs)
        elif action == "land":
            flight.land(**kwargs)
        elif action == "stop":
            flight.stop(**kwargs)
        elif action == "reload_animation":
            self.animation.on_animation_update(kwargs.get("filepath", self.animation.filepath))
        elif action == "play":
            self._play_animation(**kwargs)
        elif action == "disarm":
            flight.disarm(**kwargs)
        elif action == "reboot_fcu":
            flight.reboot_fcu(**kwargs)
        elif action == "calibrate_gyro":
            flight.calibrate_gyro(**kwargs)
        elif action == "calibrate_level":
            flight.calibrate_level(**kwargs)
        elif action == "test_leds":
            led.test(**kwargs)
        elif action == "flip":
            logger.warning("'flip' is not implemented on this airframe/firmware; ignoring")
        else:
            logger.warning("Unknown action '%s'", action)

    def _play_animation(self, start_time=None, action=None, interrupter=None):
        if self.animation.state != "OK":
            logger.error("Can't play animation, state is '%s'", self.animation.state)
            return

        current_height = flight.get_telemetry_locked().z
        run_action = action or self.animation.get_start_action(current_height)
        if run_action not in ("fly", "takeoff"):
            logger.error("Can't start animation: %s", run_action)
            return

        if start_time is not None:
            offset = network.get_time_offset()["offset_sec"]
            deadline = rospy.get_time() + (start_time - network.corrected_now(offset))
            while rospy.get_time() < deadline:
                if interrupter.is_set():
                    return
                rospy.sleep(0.05)

        for frame in self.animation.get_output_frames(run_action):
            if interrupter.is_set():
                return
            animation.execute_frame(frame, self.animation.config, interrupter=interrupter)
            rospy.sleep(frame.delay)
