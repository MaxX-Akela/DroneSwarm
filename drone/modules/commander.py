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

# Backstop for do_action(): normally task_stopped is set by the worker almost
# immediately after interrupter is set, since every blocking wait loop in
# flight/animation checks it every 0.05-0.2s. This timeout only protects
# against a stuck task that never checks the interrupter.
INTERRUPT_TIMEOUT = 2.0

def wait(deadline, interrupter, interval=0.05):
    """Block until the absolute rospy time `deadline`, or interrupter is set.

    Returns True if the deadline was reached, False if interrupted.
    """
    while not rospy.is_shutdown():
        remaining = deadline - rospy.get_time()
        if remaining <= 0:
            return True
        if interrupter.is_set():
            return False
        rospy.sleep(min(interval, remaining))
    return False


class TaskManager:

    def __init__(self, animation_path=ANIMATION_PATH):
        self.animation = animation.Animation(filepath=animation_path, config=config)
        self.current_task = None
        self.interrupter = threading.Event()
        self.task_stopped = threading.Event()
        self.task_stopped.set()

        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()

    def do_action(self, action_name, **kwargs):
        self.interrupter.set()
        if not self.task_stopped.wait(timeout=INTERRUPT_TIMEOUT):
            logger.warning("Previous task '%s' didn't stop in %.1fs, proceeding anyway",
                            action_name, INTERRUPT_TIMEOUT)
        self.interrupter.clear()
        self.current_task = (action_name, kwargs)

    def _worker(self):
        while not rospy.is_shutdown():
            if self.current_task:
                action, kwargs = self.current_task
                self.current_task = None
                kwargs["interrupter"] = self.interrupter

                self.task_stopped.clear()
                try:
                    self._dispatch(action, kwargs)
                except Exception as e:
                    logger.error("Action '%s' failed: %s", action, e)
                finally:
                    self.task_stopped.set()

            rospy.sleep(0.1)

    def _dispatch(self, action, kwargs):
        if action == "takeoff":
            flight.takeoff(**kwargs)
        elif action == "navto":
            flight.reach_point(**kwargs)
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
            task_start_time = rospy.get_time() + (start_time - network.corrected_now(offset))
            if not wait(task_start_time, interrupter):
                return
        else:
            task_start_time = rospy.get_time()

        elapsed = 0.0
        for frame in self.animation.get_output_frames(run_action):
            if not wait(task_start_time + elapsed, interrupter):
                return
            animation.execute_frame(frame, self.animation.config, interrupter=interrupter)
            elapsed += frame.delay
