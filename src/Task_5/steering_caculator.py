# MIT License 
# Copyright (c) 2023 Christos-Alexandros Tsingiropoulos

# This software contains work that was developed by Christos-Alexandros Tsingiropoulos
# as part of the Bosch Future Mobility Challenge (BFMC) competition project, which
# was organized by Bosch. This work represents Christos-Alexandros Tsingiropoulos's
# individual contribution to the larger project created by the VROOM team.

# Modified by Vo Thanh Nguyen on 2025
# - Improved lane detection and lane keeping algorithms.
# - Enhanced vehicle control for smoother navigation on both straight and curved roads.
# - Optimized steering adjustments to reduce oscillations and improve stability.
# - Improved response to varying road conditions, ensuring clearer lane recognition.

import configparser
import math
from typing import Dict, List, Tuple, Optional, Any, Union
import time

import cv2
import numpy as np

class LaneKeeping:

    def __init__(self,
                 width: int,
                 height: int,
                 logger: Any) -> None:
        self.width = width
        self.height = height
        self.log = logger

        self.angle = 0.0
        self.last_angle = 0.0
        self.prev_time = 0
        self.smoothed_speed = 0
        
        self.config = configparser.ConfigParser()
        self.config.read("config.ini")
        self._load_config()
        
        self._init_filters()
        
        self._setup_dimensions()

    def _load_config(self) -> None:
        self.print_flags: Dict[str, bool] = {
            "mid_lane": self.config["LANE_KEEPING"].getboolean("print_mid_lane"),
            "angle": self.config["LANE_KEEPING"].getboolean("print_angle"),
            "speed": self.config["LANE_KEEPING"].getboolean("print_speed"),
            "steering_arrow": self.config["LANE_KEEPING"].getboolean("print_steering_arrow")
        }
        
        self.pid: Dict[str, float] = {
            'Kp': float(self.config['PARAMS']['kp']),
            'Ki': float(self.config['PARAMS']['ki']),
            'Kd': float(self.config['PARAMS']['kd'])
        }
        
        self.sharp_turn: Dict[str, float] = {
            "max_coef": float(self.config["LANE_KEEPING"].get("max_coef_of_sharp_turn")),
            "min_coef": float(self.config["LANE_KEEPING"].get("min_coef_of_sharp_turn")),
            "factor": float(self.config["LANE_KEEPING"].get("sharp_turning_factor")),
            "max_tilt": float(self.config["LANE_KEEPING"].get("sharp_turning_max_tilt"))
        }
        
        self.speed: Dict[str, int] = {
            "min": int(self.config["PHYSICS"].get("min_speed")),
            "max": int(self.config["PHYSICS"].get("max_speed"))
        }
        
        self.speed_weight: Dict[str, float] = {
            "angle": float(self.config["LANE_KEEPING"].get("angle_weight")),
            "curvature": float(self.config["LANE_KEEPING"].get("curvature_weight")),
            "tilt": float(self.config["LANE_KEEPING"].get("tilt_weight")),
        }
        
        self.median_constant = int(self.config["LANE_KEEPING"].get("median_constant"))
        self.max_steering = int(self.config["PARAMS"].get("max_steering"))
        self.slices = int(self.config["LANE_DETECT"].get("slices"))
        self.bottom_offset = int(self.config["LANE_DETECT"].get("bottom_offset"))
        
        self.mu = float(self.config["LANE_KEEPING"].get("mu"))
        self.sigma = float(self.config["LANE_KEEPING"].get("sigma"))
        
        self.bottom_width = int(self.config["LANE_KEEPING"].get("bottom_width"))
        self.top_width = int(self.config["LANE_KEEPING"].get("top_width"))
        
        self.bottom_perc = float(self.config["LANE_DETECT"].get("bottom_perc"))
        self.peaks_min_width = int(self.config["LANE_DETECT"].get("peaks_min_width"))
        self.peaks_max_width = int(self.config["LANE_DETECT"].get("peaks_max_width"))

        self.error_weight = {
            "straight": float(self.config["LANE_KEEPING"].get("straight_error_weight")),
            "light_curve": float(self.config["LANE_KEEPING"].get("light_curve_error_weight")),
            "sharp_curve": float(self.config["LANE_KEEPING"].get("sharp_curve_error_weight"))
        } 

    def _init_filters(self) -> None:
        self.steer_value_list: List[float] = []

        self.curvature_history: List[float] = []
        self.curvature_history_size = 3

        self.integral = 0
        self.max_integral = 100
        self.prev_error = 0
        self.error_history_size = 20
        self.error_history = [0] * self.error_history_size
        
        self.max_steering_angle_change = 2.5  
        self.last_steering_angles: List[float] =  []
        
        self.a = 0
        self.offset = 10
        self.road_type = "straight"
        self.road_type_history: List[str] = ["straight"] * 5
        
        self.speed_smoothing = 0.8

    def _setup_dimensions(self) -> None:
        self.bottom_row_index = self.height - self.bottom_offset
        end = int((1 - self.bottom_perc) * self.height)
        
        self.step = int(-(self.height * self.bottom_perc / self.slices))
        self.real_slices = (end - self.bottom_row_index) // self.step
        self.top_row_index = self.bottom_row_index + self.real_slices * self.step
        
        self.plot_y = np.linspace(self.bottom_row_index, self.top_row_index, self.real_slices)
        self.plot_y_norm = np.linspace(0, 1, self.real_slices)
        
        self._poly_matrix_cache = np.vstack([self.plot_y**2, self.plot_y, np.ones_like(self.plot_y)])
        
        self.midlane_weights = self._calculate_midlane_weights()

    def _calculate_midlane_weights(self, type: str = "straight") -> np.ndarray:
        if type == "straight":
            mid_lane_weights = 0.9
        elif type == "light_curve":
            mid_lane_weights = 0.5
        else : mid_lane_weights = 0.3
        weights = np.exp(-0.5 * ((np.linspace(0, 1, self.real_slices) - mid_lane_weights) / 0.3)**2)
        weights /= weights.sum()
        return weights

    def _poly_matrix(self) -> np.ndarray:
        return self._poly_matrix_cache

    def _calculate_dynamic_lane_width(self, a: float) -> int:
        if abs(a) < self.sharp_turn["min_coef"]:
            return self.top_width
        elif self.sharp_turn["min_coef"] <= abs(a) <= self.sharp_turn["max_coef"]:
            ratio = (abs(a) - self.sharp_turn["min_coef"])/(self.sharp_turn["max_coef"] - self.sharp_turn["min_coef"])
            return int(self.top_width * (1 - ratio*(self.sharp_turn["factor"] - 1)))
        return int(self.top_width * 0.8)

    def _calculate_lane_offset(self, top_width: int) -> np.ndarray:
        return self.bottom_width - (self.bottom_width - top_width) * self.plot_y_norm

    def _estimate_curvature(self,
                          left_coef: Optional[np.ndarray],
                          right_coef: Optional[np.ndarray]) -> float:
        if left_coef is not None and right_coef is not None:
            curvature = (abs(left_coef[0]) + abs(right_coef[0])) / 2
        elif left_coef is not None:
            curvature = abs(left_coef[0])
        elif right_coef is not None:
            curvature = abs(right_coef[0])
        else:
            curvature = 0
        return curvature

    def update_curvature_history(self, curvature: float) -> None:
        self.curvature_history.append(curvature)
        if len(self.curvature_history) > self.curvature_history_size:
            self.curvature_history.pop(0)
        
        avg_curvature = np.mean(self.curvature_history) if self.curvature_history else 0
        if avg_curvature < self.sharp_turn["min_coef"]:
            estimate_road_type = "straight"
        elif avg_curvature < self.sharp_turn["max_coef"]:
            estimate_road_type = "light_curve"
        else:
            estimate_road_type = "sharp_curve"
        
        self.road_type_history.append(estimate_road_type)
        if len(self.road_type_history) > 10:
            self.road_type_history.pop(0)
        
        most_common_type = max(set(self.road_type_history[-5:]), 
                              key = self.road_type_history[-5:].count)
        
        if self.road_type_history[-5:].count(most_common_type) >= 4:
            self.road_type = most_common_type

    def calculate_mid_lane(self, left_fit, right_fit):

        curvature = self._estimate_curvature(left_fit, right_fit)
        self.update_curvature_history(curvature)
        self.update_curvature_history(curvature)
        
        if left_fit is not None and right_fit is not None:
            desire_lane = (
                (left_fit[0] + right_fit[0]) / 2 * self.plot_y**2
                + (left_fit[1] + right_fit[1]) / 2 * self.plot_y
                + (left_fit[2] + right_fit[2]) / 2
            )

        elif left_fit is None:
            desire_lane = np.ndarray([])

            top_width = self.top_width

            if right_fit[0] < -(self.sharp_turn["max_coef"]):
                top_width = int(top_width * self.sharp_turn["factor"])
            elif right_fit[0] < -(self.sharp_turn["min_coef"]):
                add = (-right_fit[0] - self.sharp_turn["min_coef"]) / (
                    self.sharp_turn["max_coef"] - self.sharp_turn["min_coef"]
                )
                top_width = int(top_width * (1 + add * (self.sharp_turn["factor"] - 1)))
            cnt = 0
            for i in self.plot_y:

                fix = self.bottom_width - ((self.bottom_width - top_width) * self.plot_y_norm[cnt])
                cnt += 1

                value = (right_fit[0] * i**2 + right_fit[1] * i + right_fit[2]) - fix

                desire_lane = np.append(desire_lane, value)

            desire_lane = desire_lane[1:]

        elif right_fit is None:
            desire_lane = np.ndarray([])
            top_width = self.top_width
            if left_fit[0] > self.sharp_turn["max_coef"]:
                top_width = int(top_width * self.sharp_turn["factor"])
            elif left_fit[0] > self.sharp_turn["min_coef"]:
                add = (left_fit[0] - self.sharp_turn["min_coef"]) / (
                    self.sharp_turn["max_coef"] - self.sharp_turn["min_coef"]
                )
                top_width = int(top_width * (1 + add * (self.sharp_turn["factor"] - 1)))

            cnt = 0
            for i in self.plot_y:
                fix = self.bottom_width - ((self.bottom_width - top_width) * self.plot_y_norm[cnt])
                cnt += 1
                value = (left_fit[0] * i**2 + left_fit[1] * i + left_fit[2]) + fix
                desire_lane = np.append(desire_lane, value)

            desire_lane = desire_lane[1:]
        
        print("curvature: ", curvature)
        return desire_lane.astype(np.int32)
    
    def get_weighted_error(self, mid_lane: np.ndarray) -> float:
        if len(mid_lane) == 0:
            return 0
        
        center = int(self.width / 2.0)
        
        weighted_mean = np.sum(mid_lane * self._calculate_midlane_weights(type = 'straight'))
        current_weight = self.error_weight["straight"]
        index = 4
        if self.road_type == "sharp_curve":
            current_weight = self.error_weight["sharp_curve"]
            index = 6
            weighted_mean = np.sum(mid_lane * self._calculate_midlane_weights(type = 'sharp_curve'))
        elif self.road_type == "light_curve":
            current_weight = self.error_weight["light_curve"]
            weighted_mean = np.sum(mid_lane * self._calculate_midlane_weights(type = 'light_curve'))
            index = 7

        
        error = (weighted_mean - center) 
        if error > 0: error -= 10
        else: error += 10
        error = np.clip(error, -160, 160)

        self.error_history.append(error)
        if len(self.error_history) > self.error_history_size:
            self.error_history.pop(0)
            
        recent_errors = self.error_history[-index:]
        weighted_history = np.mean(recent_errors) if recent_errors else 0
        
        smoothed_error = current_weight * weighted_history #+ history_weight * weighted_history
        print("weighted_mean: ", weighted_mean, "center: ", center)
        print("smooth_error: ", smoothed_error, "error: ", error)
    
        return smoothed_error

    def calculate_steering_angle(self, error: float) -> float:
        
        angle = 90 - math.degrees(math.atan2(self.height, error))
   
        if self.last_steering_angles:
            last_angle = self.last_steering_angles[-1]
            
            max_change = self.max_steering_angle_change
            
            if self.road_type == "sharp_curve":
                max_change *= 1.2
            elif self.road_type == "straight" and abs(angle) < 10:
                max_change *= 0.8
                
            change_needed = abs(angle - last_angle)
            if change_needed > max_change * 2:
                max_change *= 1.2
            
            angle = np.clip(angle, last_angle - max_change, last_angle + max_change)
        
        self.last_steering_angles.append(angle)
        if len(self.last_steering_angles) > 5:
            self.last_steering_angles.pop(0)
        
        return np.clip(angle, -self.max_steering, self.max_steering)
    
    def smooth_steering_angle(self) -> None:

        self.steer_value_list.insert(0, self.angle)
        if len(self.steer_value_list) == self.median_constant:
            self.steer_value_list.pop()

        if len(self.steer_value_list) > 0:
            alpha = 0.7
            
            if self.road_type == "straight":
                self.angle = np.clip(self.angle, -8, 8)
            elif self.road_type == "sharp_curve":
                self.angle = np.clip(self.angle, -20, 20)
            else:
                self.angle = np.clip(self.angle, -15, 15)
        self.last_angle = self.angle

    def calculate_speed(self,
                      angle: float,
                      left_coef: Optional[np.ndarray],
                      right_coef: Optional[np.ndarray]) -> float:
 
        avg_curvature = np.mean(self.curvature_history) if self.curvature_history else 0
        angle_factor = 1.0 - min(1.0, abs(angle) / self.max_steering) * self.speed_weight["angle"]
        
        curve_factor = 1.0 - min(1.0, avg_curvature / self.sharp_turn["max_coef"]) * self.speed_weight["curvature"]
        
        target_speed = self.speed["min"] + (self.speed["max"] - self.speed["min"]) * min(angle_factor, curve_factor)
        
        self.smoothed_speed = self.speed_smoothing * self.smoothed_speed + (1 - self.speed_smoothing) * target_speed
        
        return self.smoothed_speed

    def controller(self,
                 left: Optional[np.ndarray],
                 right: Optional[np.ndarray],
                 frame: Optional[np.ndarray] = None) -> Tuple[float, float, Optional[np.ndarray]]:
        print(self.road_type)
        if left is None and right is None:
            self.log.warning("Không tìm thấy làn đường")
            decay_factor = 0.9
            decayed_angle = self.last_angle * decay_factor
            return decayed_angle, 0, frame
            
        mid_lane = self.calculate_mid_lane(left, right)
        
        if self.print_flags["mid_lane"] and frame is not None:
            self._visualize_mid_lane(frame, mid_lane)
            
        error = self.get_weighted_error(mid_lane)
        
        steering_angle = self.calculate_steering_angle(error)
        
        return steering_angle, error, frame

    def lane_keeping(self, lanes_detection: Dict[str, Any]) -> Tuple[float, float, Optional[np.ndarray]]:

        frame = lanes_detection["frame"]
        left_coef = lanes_detection["left_coef"]
        right_coef = lanes_detection["right_coef"]
        
        trust_left = lanes_detection["trust_left"]
        trust_right = lanes_detection["trust_right"]

        # left_coef = left_coef if trust_left else None
        # right_coef = right_coef if trust_right else None

        self.angle, _, frame = self.controller(left_coef, right_coef, frame)

        self.smooth_steering_angle()

        speed = self.calculate_speed(self.angle, left_coef, right_coef)

        if frame is not None:
            if self.print_flags["angle"]:
                self._visualize_angle_param(frame, self.angle)
            if self.print_flags["speed"]:
                self._visualize_speed_param(frame, speed)
            if self.print_flags["steering_arrow"]:
                self._visualize_steering_arrow(frame, self.angle)

        return self.angle, speed, frame
    
    def _visualize_mid_lane(self, frame, plot_x, bgr_colour=(50, 205, 50)):

        if frame is not None:
            plot_y = self.plot_y.astype(np.int32)
            for i in range(1, len(plot_y) - 1):

                new = (int(plot_x[i]), int(plot_y[i]))
                prev = (int(plot_x[i - 1]), int(plot_y[i - 1]))

                cv2.line(frame, prev, new, bgr_colour, thickness=3)
                cv2.circle(frame, center = new, radius = 2, color = (255, 0, 255), thickness = 2)

    def _visualize_angle_param(self, frame, angle):
        
        cv2.putText(frame, f"Angle: {angle:.2f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 155, 100), 2)

    def _visualize_speed_param(self, frame, speed):
        
        cv2.putText(frame, f"Speed: {speed:.2f} m/s", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (145, 100, 255), 2)

    
    def _visualize_steering_arrow(self, frame, angle):

        center_x = self.width // 2
        center_y = self.height - 30
        
        arrow_length = 120
        
        angle_rad = math.radians(angle)
        end_x = int(center_x + arrow_length * math.sin(angle_rad))
        end_y = int(center_y - arrow_length * math.cos(angle_rad))
        
        angle_abs = abs(angle)
        max_angle = int(self.config["PARAMS"].get("max_steering"))
        intensity = min(1.0, angle_abs / max_angle)
        
        arrow_color = (
            int(255 * intensity),  # Blue component increases with angle
            int(255 * (1 - intensity)),  # Green component decreases with angle
            0  # Red component
        )
        
        if angle_abs < 0.5:
            arrow_color = (0, 255, 0)  # Green when straight
        
        cv2.arrowedLine(frame, (center_x, center_y), (end_x, end_y), 
                        arrow_color, 4, tipLength=0.3)
        
        cv2.circle(frame, (center_x, center_y), 8, (0, 0, 255), -1)