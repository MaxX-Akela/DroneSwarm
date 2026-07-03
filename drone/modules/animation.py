import csv
import copy
import math
import time
import numpy
import logging
import threading
from typing import List, Optional

logger = logging.getLogger(__name__)

try:
    import modules.flight as flight
except ImportError:
    print("Can't import flight control module!")

try:
    import modules.led as led
except ImportError:
    print("Can't import led control module!")

interrupt_event = threading.Event()

def moving(f1, f2, delta, x=True, y=True, z=True):
    return ((abs(f1.x - f2.x) > delta) and x
        or  (abs(f1.y - f2.y) > delta) and y
        or  (abs(f1.z - f2.z) > delta) and z)

def get_numbers(frames):
    return [frame.number for frame in frames]

def get_actions(frames):
    return [frame.action for frame in frames]

def get_delays(frames):
    return [frame.delay for frame in frames]

def get_stats(frames):
    return [[f.number, f.action, f.delay] for f in frames]

def get_table(frames, header):
    table = []
    for frame in frames:
        table.append([getattr(frame, name) for name in header])
    return table

def get_default_header():
    return ["number", "action", "delay", "x", "y", "z", "yaw", "red", "green", "blue"]

def get_start_frame_index(frames):
    for i, frame in enumerate(frames):
        if frame.action != 'stand':
            return i
    return len(frames)

def get_duration(frames):
    return sum(frame.delay for frame in frames)


class Frame:
    params_dict = {
        "number": None,
        "action": 'fly', 
        "x": None,
        "y": None,
        "z": None,
        "yaw": None,
        "red": None,
        "green": None,
        "blue": None,
        "delay": 0,
    }

    def __init__(self, csv_row=None, delay=None):
        for key, value in self.params_dict.items():
            setattr(self, key, value)
        if csv_row:
            self.load_csv_row(csv_row)
        if delay is not None:
            self.delay = delay

    def load_csv_row(self, csv_row):
        number, x, y, z, yaw, red, green, blue = csv_row
        self.number = int(number)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.yaw = float(yaw)
        self.red = int(red)
        self.green = int(green)
        self.blue = int(blue)

    def get_pos(self):
        if None in [self.x, self.y, self.z]:
            return []
        return [self.x, self.y, self.z]

    def get_color(self):
        if None in [self.red, self.green, self.blue]:
            return []
        return [self.red, self.green, self.blue]

    def set_yaw(self, yaw):
        if yaw != "animation":
            self.yaw = math.radians(float(yaw))

    def pose_is_valid(self):
        return bool(self.get_pos()) and (self.yaw is not None)


class Animation:
    def __init__(self, filepath="animation.csv", config=None):
        self.lock = threading.Lock()
        with self.lock:
            self.reset(filepath, config)
            if config is not None:
                self.on_animation_update(self.filepath)

    def reset(self, filepath, config):
        self.id = None
        self.original_frames = []
        self.transformed_frames = []
        self.static_begin_index = 0
        self.takeoff_index = 0
        self.route_index = 0
        self.land_index = 0
        self.static_end_index = 0
        self.output_frames = []
        self.output_frames_min_z = None
        self.output_frames_takeoff = []
        self.output_frames_takeoff_min_z = None
        self.start_time = None
        self.start_frame_index = 0
        self.filepath = filepath
        self.config = config
        self.state = None

    def set_state(self, state, log_error=False):
        self.state = state
        if log_error:
            logger.error(state)

    def load(self):
        self.set_state("OK")
        try:
            delay = self.config.animation_frame_delay
        except (AttributeError, KeyError):
            self.set_state("Bad animation delay from config", log_error=True)
            return

        try:
            with open(self.filepath, 'r', encoding='utf-8') as animation_file:
                current_frame_delay = delay
                csv_reader = csv.reader(animation_file, delimiter=',', quotechar='|')
                
                try:
                    row_0 = next(csv_reader)
                except StopIteration:
                    self.set_state("Animation file is empty", log_error=True)
                    return

                if len(row_0) == 1:
                    self.id = row_0[0]
                    logger.debug(f"Got animation_id: {self.id}")
                elif len(row_0) == 2:
                    try:
                        current_frame_delay = float(row_0[1])
                        logger.debug(f"Got new frame delay: {current_frame_delay}")
                    except ValueError as e:
                        self.set_state(f"Can't parse delay: {e}", log_error=True)
                        return
                else:
                    self.id = "No animation id"
                    try:
                        self.original_frames.append(Frame(row_0, current_frame_delay))
                    except ValueError as e:
                        self.set_state(f"Can't parse frame: {e}", log_error=True)
                        return

                for row in csv_reader:
                    if not row: continue
                    if len(row) == 2:
                        try:
                            current_frame_delay = float(row[1])
                        except ValueError: continue
                    else:
                        try:
                            self.original_frames.append(Frame(row, current_frame_delay))
                        except ValueError as e:
                            self.set_state(f"Can't parse frame row: {e}", log_error=True)
                            return
            
            self.set_yaw()
            if self.state == "OK":
                if self.original_frames:
                    self.split()
                else:
                    self.set_state("No frames loaded!", log_error=True)

        except IOError:
            self.set_state(f"File {self.filepath} can't be opened", log_error=True)

    def set_yaw(self):
        try:
            yaw = self.config.animation_yaw
            for frame in self.original_frames:
                frame.set_yaw(yaw)
        except (AttributeError, KeyError, ValueError) as e:
            self.set_state(f"Can't set yaw: {e}", log_error=True)

    def split(self, move_delta=0.01):
        if not self.original_frames: return
        
        frames = copy.deepcopy(self.original_frames)
        n = len(frames)
        
        i = 0
        while i < n - 1 and not moving(frames[i], frames[i+1], move_delta):
            i += 1
        self.takeoff_index = i + 1 if i > 0 else 0

        i = self.takeoff_index
        while i < n - 1:
            if moving(frames[i], frames[i+1], move_delta, z=False) or (frames[i+1].z - frames[i].z <= 0):
                break
            i += 1
        self.route_index = i + 1 if i > self.takeoff_index else self.takeoff_index

        i = n - 1
        while i > 0 and not moving(frames[i], frames[i-1], move_delta):
            i -= 1
        self.static_end_index = i

        while i > 0:
            if moving(frames[i], frames[i-1], move_delta, z=False) or (frames[i-1].z - frames[i].z <= 0):
                break
            i -= 1
        self.land_index = i

    def transform(self):
        try:
            offsets = numpy.array(self.config.animation_common_offset) + numpy.array(self.config.animation_private_offset)
            x0, y0, z0 = offsets
            x_ratio, y_ratio, z_ratio = self.config.animation_ratio
        except (AttributeError, ValueError, KeyError):
            self.set_state("Transform error: check offsets/ratio in config", log_error=True)
            return

        self.transformed_frames = copy.deepcopy(self.original_frames)
        for frame in self.transformed_frames:
            frame.x = x_ratio * frame.x + x0
            frame.y = y_ratio * frame.y + y0
            frame.z = z_ratio * frame.z + z0

    def mark_stand_frames(self):
        if not self.transformed_frames: return
        try:
            takeoff_level = self.config.animation_takeoff_level
        except (AttributeError, KeyError):
            self.set_state("Marking error: no takeoff_level", log_error=True)
            return

        action = "stand" if self.transformed_frames[0].z < takeoff_level else "fly"
        for frame in self.transformed_frames[:self.takeoff_index]:
            frame.action = action

        action = "stand" if self.transformed_frames[-1].z < takeoff_level else "fly"
        for frame in self.transformed_frames[self.static_end_index:]:
            frame.action = action

    def apply_flags(self):
        self.output_frames = []
        self.output_frames_takeoff = []
        if not self.transformed_frames: return

        try:
            conf = self.config
            flags = [
                (conf.animation_output_static_begin, 0, self.takeoff_index),
                (conf.animation_output_takeoff, self.takeoff_index, self.route_index),
                (conf.animation_output_route, self.route_index, self.land_index),
                (conf.animation_output_land, self.land_index, self.static_end_index),
                (conf.animation_output_static_end, self.static_end_index, None)
            ]
            takeoff_level = conf.animation_takeoff_level
        except (AttributeError, KeyError):
            self.set_state("Output flags error", log_error=True)
            return

        all_f = copy.deepcopy(self.transformed_frames)
        for enabled, start, end in flags:
            if enabled:
                chunk = all_f[start:end]
                self.output_frames += chunk
                # Специальная логика для взлета
                if start == self.takeoff_index:
                    if all_f[self.takeoff_index].z >= takeoff_level:
                        self.output_frames_takeoff += chunk
                else:
                    self.output_frames_takeoff += chunk

        if self.output_frames:
            self.output_frames_min_z = min(f.z for f in self.output_frames)
        if self.output_frames_takeoff:
            self.output_frames_takeoff_min_z = min(f.z for f in self.output_frames_takeoff)

    def mark_flight(self):
        if not self.output_frames: return
        try:
            c = self.config
            arming_time = c.flight_arming_time
            takeoff_time = c.flight_takeoff_time
            rfp_time = c.flight_reach_first_point_time
            land_delay = c.flight_land_delay
        except (AttributeError, KeyError):
            self.set_state("Flight marking error", log_error=True)
            return

        for i, frame in enumerate(self.output_frames):
            if frame.action == 'fly':
                new_f = copy.deepcopy(frame)
                new_f.action, new_f.delay = 'arm', arming_time
                self.output_frames.insert(i, new_f)
                break

        for i, frame in enumerate(self.output_frames_takeoff):
            if frame.action == 'fly':
                frame.action, frame.delay = 'reach', rfp_time
                new_f = copy.deepcopy(frame)
                new_f.action, new_f.delay = 'takeoff', takeoff_time
                self.output_frames_takeoff.insert(i, new_f)
                break

        # Вставка LAND (основной)
        for i in range(len(self.output_frames)-1, -1, -1):
            if self.output_frames[i].action == 'fly':
                new_f = copy.deepcopy(self.output_frames[i])
                new_f.delay = land_delay
                self.output_frames[i].action = 'land'
                self.output_frames.insert(i, new_f)
                break
        
        # Вставка LAND (takeoff)
        for i in range(len(self.output_frames_takeoff)-1, -1, -1):
            if self.output_frames_takeoff[i].action == 'fly':
                new_f = copy.deepcopy(self.output_frames_takeoff[i])
                new_f.delay = land_delay
                self.output_frames_takeoff[i].action = 'land'
                self.output_frames_takeoff.insert(i, new_f)
                break

        self.start_frame_index = get_start_frame_index(self.output_frames)
        self.start_time = get_duration(self.output_frames[:self.start_frame_index])

    def on_animation_update(self, filepath="animation.csv", config=None):
        if config is None: config = self.config
        self.reset(filepath, config)
        self.load()
        if self.original_frames:
            self.on_config_update(self.config)

    def on_config_update(self, config):
        self.config = config
        self.transform()
        self.mark_stand_frames()
        self.apply_flags()
        self.mark_flight()

    def get_start_frame(self, action):
        try:
            if action == 'fly': return self.output_frames[self.start_frame_index]
            if action == 'takeoff': return self.output_frames_takeoff[self.start_frame_index]
        except IndexError: return None

    def get_output_frames(self, action):
        return self.output_frames if action == 'fly' else (self.output_frames_takeoff if action == 'takeoff' else [])

    def get_min_z(self, action):
        return self.output_frames_min_z if action == 'fly' else (self.output_frames_takeoff_min_z if action == 'takeoff' else [])

    def get_start_action(self, current_height=0, state="STANDBY", tolerance=0.2):
        if not self.output_frames: return 'error: empty output frames'
        
        c = self.config
        if c.animation_check_ground and math.isnan(current_height):
            return 'error: bad copter height'

        try:
            start_action = c.animation_start_action
            takeoff_level = c.animation_takeoff_level
        except (AttributeError, KeyError):
            return 'error in config [ANIMATION]'

        if start_action == 'auto':
            start_action = 'takeoff' if self.output_frames[0].z > takeoff_level else 'fly'
        
        if start_action not in ('takeoff', 'fly'):
            return 'error in [ANIMATION] start_action parameter'

        if c.animation_check_ground:
            try:
                g_level = current_height if c.animation_ground_level == 'current' else float(c.animation_ground_level)
            except (ValueError, KeyError):
                return 'error in [ANIMATION] ground_level'
            
            min_z = self.get_min_z(start_action)
            if state != "ACTIVE" and (g_level - tolerance) > min_z:
                return f'error: animation is lower than ground level for {g_level - min_z:.2f}m'
        
        return start_action

# Функции выполнения полета
def execute_frame(frame, config, interrupter=interrupt_event):
    auto_arm = False
    use_leds = config.led_use
    f_id = config.flight_frame_id
    
    if frame.action == 'takeoff':
        use_leds &= config.led_takeoff_indication
        takeoff(z=config.flight_takeoff_height, frame_id=f_id, timeout=config.flight_takeoff_time, use_leds=use_leds, interrupter=interrupter)
        return
    
    if frame.action == 'land':
        use_leds &= config.led_land_indication
        land(frame_id=f_id, timeout=config.flight_land_timeout, use_leds=use_leds, interrupter=interrupter)
        return
    
    if frame.action in ('fly', 'arm'):
        auto_arm = (frame.action == 'arm')
        if frame.pose_is_valid():
            flight.navto(x=frame.x, y=frame.y, z=frame.z, yaw=frame.yaw, frame_id=f_id, auto_arm=auto_arm, interrupter=interrupter)
        else:
            logger.error("Frame pose is not valid for flying")
            
    if use_leds:
        color = frame.get_color()
        if color:
            led.set_effect(r=color[0], g=color[1], b=color[2])
        else:
            logger.error("Can't get frame color!")

def turn_off_led(interrupter=interrupt_event):
    led.set_effect(r=0, g=0, b=0)

def takeoff(z=1.5, safe_takeoff=False, frame_id='map', timeout=5.0, use_leds=True, interrupter=interrupt_event):
    if use_leds:
        led.set_effect(effect='wipe', r=255, g=0, b=0)
    
    result = flight.takeoff(height=z, timeout_takeoff=timeout, frame_id=frame_id, emergency_land=safe_takeoff, interrupter=interrupter)
    
    if result == 'not armed':
        raise Exception('STOP')
    
    if use_leds:
        led.set_effect(effect='blink_fast', r=0, g=255, b=0)

def land(z=1.5, descend=False, timeout=5.0, frame_id='map', use_leds=True, interrupter=interrupt_event):
    led.set_effect(r=0, g=0, b=0)
    if use_leds:
        led.set_effect(effect='blink_fast', r=255, g=0, b=0)
    
    flight.land(z=z, descend=descend, timeout_land=timeout, frame_id_land=frame_id, interrupter=interrupter)
    
    if use_leds:
        while flight.get_telemetry_locked().armed:
            if interrupter.is_set(): break
            time.sleep(0.5)
        led.set_effect(r=0, g=0, b=0)