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

import cv2
import numpy as np
from scipy.signal import find_peaks


class LaneDetection:
    def __init__(self, width, height, lk):
        self.width, self.height, self.lk = width, height, lk
        self.config = configparser.ConfigParser()
        self.config.read("config.ini")
        self._load_config()
        self.set_up()

    def _load_config(self):
        ld = self.config["LANE_DETECT"]
        
        # Core detection parameters
        self.custom_find_peaks = ld.getboolean("custom_find_peaks")
        self.slices = int(ld.get("slices"))
        self.min_peaks_for_lane = int(ld.get("min_peaks_for_lane"))
        self.bottom_offset = int(ld.get("bottom_offset"))
        self.prev_lane_coeffs = {"left": None, "right": None}
        self.prev_trust_lk = False
        self.hor_step_by_step = True
        
        # Visualization flags
        self.print_flags = {
            "lanes": ld.getboolean("print_lanes"),
            "peaks": ld.getboolean("print_peaks"),
            "certainty": ld.getboolean("print_lane_certainty"),
            "dashed": ld.getboolean("print_if_dashed"),
            "peaks_detection": ld.getboolean("print_peaks_detection")
        }
        
        # Percentage parameters
        self.perc_params = {
            "optimal_peak": float(ld.get("optimal_peak_perc")),
            "min_lane_dist": float(ld.get("min_lane_dist_perc")),
            "max_lane_dist": float(ld.get("max_lane_dist_perc")),
            "allowed_certainty_dif": float(ld.get("allowed_certainty_perc_dif")),
            "certainty_from_peaks": float(ld.get("certainty_perc_from_peaks")),
        }
        
        
        # Coefficient constraints
        self.extreme_coefs = {
            "second_deg": float(ld.get("extreme_coef_second_deg")),
            "first_deg": float(ld.get("extreme_coef_first_deg")),
        }
        
        # Certainty thresholds
        self.min_certainty = {
            "single": int(ld.get("min_single_lane_certainty")),
            "dual": int(ld.get("min_dual_lane_certainty")),
        }
        
        # Square pulse detection parameters
        self.square_pulses = {
            "min_height": int(ld.get("square_pulses_min_height")),
            "pix_dif": int(ld.get("square_pulses_pix_dif")),
            "min_height_dif": int(ld.get("square_pulses_min_height_dif")),
            "allowed_peaks_width_error": int(ld.get("square_pulses_allowed_peaks_width_error")),
        }
        self.minimum = self.square_pulses["min_height"]
        
        # Width and distance parameters
        self.max_allowed_dist = float(ld.get("max_allowed_width_perc")) * self.width
        self.weights = {
            "width_distance": float(ld.get("weight_for_width_distance")),
            "expected_value_distance": float(ld.get("weight_for_expected_value_distance")),
        }

    def set_up(self):
        ld = self.config["LANE_DETECT"]
        lk = self.config["LANE_KEEPING"]
        
        # Horizontal detection flag
        self.is_horizontal = False
        
        # Row indices and step calculation
        self.bottom_perc = float(ld.get("bottom_perc"))
        self.bottom_row_index = self.height - self.bottom_offset
        end = int((1 - self.bottom_perc) * self.height)
        self.step = int(-(self.height * self.bottom_perc / self.slices))
        self.real_slices = (end - self.bottom_row_index) // self.step
        self.top_row_index = self.bottom_row_index + self.real_slices * self.step
        self.height_norm = np.linspace(0, 1, self.real_slices + 1)
        
        # Peak width constraints
        self.peaks_width = {
            "min": int(ld.get("peaks_min_width")),
            "max": int(ld.get("peaks_max_width")),
        }
        
        # Lane width parameters
        self.width_params = {
            "bottom": int(lk.get("bottom_width")),
            "top": int(lk.get("top_width")),
        }
        
        # Width difference constraints
        self.min_top_width_dif = self.width_params["top"] * self.perc_params["min_lane_dist"]
        self.max_top_width_dif = self.width_params["top"] * self.perc_params["max_lane_dist"]
        self.min_bot_width_dif = self.width_params["bottom"] * self.perc_params["min_lane_dist"]
        self.max_bot_width_dif = self.width_params["bottom"] * self.perc_params["max_lane_dist"]
        
        # Dashed lane detection parameters
        self.dashed_params = {
            "max_points": float(ld.get("dashed_max_dash_points_perc")) * self.real_slices,
            "min_points": float(ld.get("dashed_min_dash_points_perc")) * self.real_slices,
            "min_space_points": float(ld.get("dashed_min_space_points_perc")) * self.real_slices,
            "min_count": float(ld.get("dashed_min_count_of_dashed_lanes")),
        }
        
        # Shorthand references for frequent access
        self.max_points = self.dashed_params["max_points"]
        self.min_points = self.dashed_params["min_points"]
        self.min_points_space = self.dashed_params["min_space_points"]
        self.min_peaks_threshold = 10

    def visualize_peaks_detection(self, img, lanes, peaks):
        visualization = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        PEAK_COLOR = (0, 255, 0)  
        LANE_COLORS = [(255, 0, 0), (0, 0, 255), (255, 255, 0)] 

        for (x, y) in peaks:
            cv2.circle(visualization, (x, y), radius = 3, color = PEAK_COLOR, thickness = 3)
            cv2.line(visualization, (x, 0), (x, visualization.shape[0]), PEAK_COLOR, thickness = 2)

        for lane_idx, lane in enumerate(lanes):
            lane_color = LANE_COLORS[lane_idx % len(LANE_COLORS)] 
            for i in range(len(lane) - 1):
                start_point = tuple(lane[i][:2])       
                end_point = tuple(lane[i + 1][:2])  
                cv2.line(visualization, start_point, end_point, lane_color, thickness=2)

        cv2.imshow("gray", visualization)

    def lanes_detection(self, img: np.ndarray) -> dict:
        lanes, peaks, gray = self.peaks_detection(img)

        if self.print_flags["peaks_detection"]:
            self.visualize_peaks_detection(gray, lanes, peaks)

        left_lane, right_lane = self.choose_correct_lanes(lanes)
        
        self._update_dynamic_params(gray, left_lane, right_lane)

        left_coef, right_coef = self.create_lanes_from_peaks(img, left_lane, right_lane)

        processed_data = self.lanes_post_processing(
            img, left_coef, left_lane, right_coef, right_lane
        )
        left_coef, right_coef, l_perc, r_perc, trust_l, trust_r, trust_lk = processed_data

        self._update_geometry_params(left_coef, right_coef, trust_l, trust_r)

        
        if self.print_flags["peaks"]:
            self.visualize_all_peaks(img, peaks)
            self.visualize_peaks(img, left_lane, right_lane)

        return {
            "frame": img,
            "left": left_lane,
            "right": right_lane,
            "left_coef": left_coef,
            "right_coef": right_coef,
            "l_perc": l_perc,
            "r_perc": r_perc,
            "trust_left": trust_l,
            "trust_right": trust_r,
            "trust_lk": trust_lk,
        }

    def _update_dynamic_params(self, gray: np.ndarray, left: list, right: list) -> None:
        if left is None or right is None:
            return

        intensities = [
            gray[y, x] for x, y in left + right
            if 0 <= y < gray.shape[0] and 0 <= x < gray.shape[1]
        ]
        
        if intensities:
            min_intensity = min(intensities)
            new_min_height = np.clip(min_intensity - 20, self.minimum, 170)
            
            if len(left) + len(right) < self.min_peaks_threshold:
                new_min_height = self.minimum
            
            self.square_pulses["min_height"] = new_min_height

    def _update_geometry_params(self, left_coef: np.ndarray, right_coef: np.ndarray, 
                            trust_left: bool, trust_right: bool) -> None:
        
        if (left_coef is None or right_coef is None or 
            not isinstance(left_coef, np.ndarray) or 
            not isinstance(right_coef, np.ndarray) or 
            not trust_left or not trust_right):
            return

        def calculate_points(coeffs):
            a, b, c = coeffs
            return (
                int(a * self.bottom_row_index**2 + b * self.bottom_row_index + c),
                int(a * self.top_row_index**2 + b * self.top_row_index + c)
            )

        left_bottom, left_top = calculate_points(left_coef)
        right_bottom, right_top = calculate_points(right_coef)

        self.width_params.update({
            "bottom": (right_bottom - left_bottom) // 2,
            "top": (right_top - left_top) // 2
        })
        
        self.lk.bottom_width = self.width_params["bottom"]
        self.lk.top_width = self.width_params["top"]


    def visualize_horizontal_line(self, frame, detected_line_segment):
        if detected_line_segment:
            cv2.line(frame, detected_line_segment[0], detected_line_segment[1], (143, 188, 143), thickness=5)


    def horizontal_detection(self, frame, max_allowed_slope=0.25):
        main_point, line = self.detect_main_point(frame, max_allowed_slope)
        detected_line_segment = self.detect_lane_line_endpoints(frame, main_point, line, max_allowed_slope)
        
        hor_min_width_dist = 0.2 * self.width
        hor_exists = (detected_line_segment and 
                    (detected_line_segment[1][0] - detected_line_segment[0][0]) > hor_min_width_dist)

        return detected_line_segment, line, hor_exists


    def detect_main_point(self, frame, max_allowed_slope):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        region_of_interest_perc = 0.4
        start_of_ROI_perc = (1 - region_of_interest_perc) / 2
        end_of_ROI_perc = 1 - start_of_ROI_perc
        width_start = int(self.width * start_of_ROI_perc)
        width_end = int(self.width * end_of_ROI_perc)
        
        top_height_perc = 0.5
        bot_height_perc = 0.95
        height_start = int(top_height_perc * self.height)
        height_end = int(bot_height_perc * self.height)
        
        main_point = None
        line = None
        max_iterations = 3

        for iterations in range(1, max_iterations):
            num_slices = 2**iterations + 1
            widths_of_slices = np.linspace(width_start, width_end, num_slices)
            
            widths_of_slices = widths_of_slices[1::2]

            for slice_width in widths_of_slices:
                slice_width = int(slice_width)
                
                tmp = [img[h][slice_width] for h in range(height_start, height_end)]
                
                ps = self.find_lane_peaks(
                    slice = tmp,
                    height_norm = 0,
                    min_height = self.square_pulses["min_height"],
                    min_height_dif = self.square_pulses["min_height_dif"],
                    pix_dif = self.square_pulses["pix_dif"],
                    allowed_peaks_width_error = self.square_pulses["allowed_peaks_width_error"],
                )

                for point in ps:
                    base_point = {"height": point + height_start, "width": slice_width}
                    
                    point_l = self.search_for_near_point(frame, img, base_point, max_allowed_slope, 
                                                    "left", width_step_perc=0.015)
                    point_r = self.search_for_near_point(frame, img, base_point, max_allowed_slope, 
                                                    "right", width_step_perc=0.015)

                    if point_l is not None and point_r is not None:
                        slope_l = (base_point["height"] - point_l["height"]) / (base_point["width"] - point_l["width"])
                        slope_r = (base_point["height"] - point_r["height"]) / (base_point["width"] - point_r["width"])
                        slope = (slope_l + slope_r) / 2
                        num_of_slopes = 2
                    elif point_l is not None:
                        slope = (base_point["height"] - point_l["height"]) / (base_point["width"] - point_l["width"])
                        num_of_slopes = 1
                    elif point_r is not None:
                        slope = (base_point["height"] - point_r["height"]) / (base_point["width"] - point_r["width"])
                        num_of_slopes = 1
                    else:
                        continue
                    
                    if abs(slope) > max_allowed_slope:
                        if self.hor_step_by_step:
                            cv2.circle(frame, (base_point["width"], base_point["height"]), 2, (0, 255, 0), 2)
                        continue

                    if (not line or 
                        (num_of_slopes > line["num_of_slopes"]) or 
                        (num_of_slopes == line["num_of_slopes"] and abs(line['slope']) > abs(slope))):
                        
                        intercept = base_point["height"] - slope * base_point["width"]
                        line = {"slope": slope, "intercept": intercept, "num_of_slopes": num_of_slopes}
                        main_point = base_point.copy()

                    if self.hor_step_by_step:
                        cv2.circle(frame, (base_point["width"], base_point["height"]), 2, (0, 0, 0), 2)

                if self.hor_step_by_step:
                    cv2.line(frame, (slice_width, height_start), (slice_width, height_end), (60, 20, 220))

            if main_point:
                return main_point, line
        
        return None, None


    def detect_lane_line_endpoints(self, frame, main_point, line, max_allowed_slope):
        if not main_point:
            return None
        
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        left_side_points = [main_point]
        right_side_points = [main_point]

        while True:
            point_l = self.search_for_near_point(frame, img, left_side_points[-1], 
                                            max_allowed_slope, "left", line=line)
            if not point_l:
                break
            left_side_points.append(point_l)

        while True:
            point_r = self.search_for_near_point(frame, img, right_side_points[-1], 
                                            max_allowed_slope, "right", line=line)
            if not point_r:
                break
            right_side_points.append(point_r)

        left_endpoint = left_side_points[-1]
        right_endpoint = right_side_points[-1]
        
        width_dif = right_endpoint["width"] - left_endpoint["width"]
        if width_dif == 0:
            return None
        
        slope = (right_endpoint["height"] - left_endpoint["height"]) / width_dif
        if abs(slope) > max_allowed_slope:
            return None

        x1 = (left_endpoint['width'], left_endpoint['height'])
        x2 = (right_endpoint['width'], right_endpoint['height'])
        
        return (x1, x2)


    def search_for_near_point(self, frame, gray_frame, base_point, max_allowed_slope, diraction="right", 
                            width_step_perc=0.03, line=None):
        max_allowed_height_dif_from_line = 0.03 * self.height
        operator = 1 if diraction == "right" else -1
        
        width_step = int(width_step_perc * self.width)
        sliding_window_height = int(0.15 * self.height)
        
        width = int(base_point['width'] + operator * width_step)
        height = base_point['height'] if not line else line["slope"] * width + line["intercept"]
        start = int(height - sliding_window_height / 2)
        end = int(height + sliding_window_height / 2)

        if (width < 0 or width >= self.width) or (start < 0 or end >= self.height):
            return None

        tmp = [gray_frame[h][width] for h in range(start, end)]
        
        ps = self.find_lane_peaks(
            slice = tmp,
            height_norm = 0,
            min_height = self.square_pulses["min_height"],
            min_height_dif = self.square_pulses["min_height_dif"],
            pix_dif = self.square_pulses["pix_dif"],
            allowed_peaks_width_error = self.square_pulses["allowed_peaks_width_error"],
        )
        
        if self.hor_step_by_step:
            cv2.line(frame, (width, start), (width, end), (60, 20, 220))
        
        if not ps:
            return None
        
        if len(ps) == 1:
            detection_height = ps[0]
        else:
            min_index = 0
            for i in range(1, len(ps)):
                if (abs(ps[min_index] + start - base_point['height']) >= 
                    abs(ps[i] + start - base_point['height'])):
                    min_index = i
            detection_height = ps[min_index]
        
        point_height = detection_height + start
        
        slope_with_base = abs((base_point["height"] - point_height) / (base_point["width"] - width))
        is_height_accepted = True if not line else abs(point_height - height) <= max_allowed_height_dif_from_line

        if slope_with_base <= max_allowed_slope and is_height_accepted:
            point = {'height': point_height, "width": width}
            if self.hor_step_by_step:
                cv2.circle(frame, (point["width"], point["height"]), 2, (0, 0, 0), 2)
            return point
        elif self.hor_step_by_step:
            cv2.circle(frame, (width, point_height), 2, (0, 250, 0), 2)
        
        return None

    def peaks_detection(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        peaks, lanes, cnt = [], [], 0

        for height in range(self.bottom_row_index, self.top_row_index - 1, self.step):
            row_data = list(map(int, img[height]))
            height_norm = self.height_norm[cnt] if not self.is_horizontal else 0

            ps = self.find_lane_peaks(
                row_data,
                height_norm,
                self.square_pulses["min_height"],
                self.square_pulses["min_height_dif"],
                self.square_pulses["pix_dif"],
                self.square_pulses["allowed_peaks_width_error"],
            )

            if not self.is_horizontal:
                cnt += 1

            peaks.extend([[point, height] for point in ps])
            lanes = self.peaks_clustering(ps, height, lanes)

            cv2.line(frame, (0, height), (self.width, height), (60, 20, 220))

        return lanes, peaks, img

    def choose_correct_lanes(self, lanes):
        lanes = [lane for lane in lanes if len(lane) >= self.min_peaks_for_lane]
        left = right = []
        optimal_threshold = self.slices * self.perc_params["optimal_peak"]

        for lane in lanes:
            is_left = lane[0][0] <= self.width / 2
            lane_length = len(lane)
            
            if is_left:
                if not left or lane_length > len(left) or lane_length > optimal_threshold:
                    left = lane
            else:
                if not right or lane_length > optimal_threshold:
                    right = lane
                    if lane_length > optimal_threshold:
                        break

        if left and right:
            (left_top, left_bot), (right_top, right_bot) = self.calculate_lane_boundaries(left), self.calculate_lane_boundaries(right)
            top_dif, bot_dif = (right_top - left_top)/2, (right_bot - left_bot)/2
            
            if not (self.min_top_width_dif < top_dif < self.max_top_width_dif and 
                    self.min_bot_width_dif < bot_dif < self.max_bot_width_dif):
                left, right = (left, []) if len(left) > len(right) else ([], right)

        return left, right

    def visualize_all_peaks(self, frame, peaks, bgr_colour=(255, 0, 255)):
        for peak in peaks:
            point = (peak[0], peak[1])
            cv2.circle(frame, point, 2, bgr_colour, 2)

        return
    
    def create_lanes_from_peaks(self, frame, left, right):
        left_coef = self._fit_polyfit(left)
        right_coef = self._fit_polyfit(right)

        if self.print_flags.get("lanes", False):
            self.visualize_lane(left_coef, frame, bgr_colour=(255, 128, 0))
            self.visualize_lane(right_coef, frame, bgr_colour=(0, 128, 255))

        return left_coef, right_coef

    def lanes_post_processing(self, frame, left_coef, left, right_coef, right, allowed_difference=None):
        allowed_difference = allowed_difference or self.perc_params["allowed_certainty_dif"]

        l_perc = self.find_lane_certainty(left_coef, self.prev_lane_coeffs["left"], left)
        r_perc = self.find_lane_certainty(right_coef, self.prev_lane_coeffs["right"], right)


        if self.print_flags.get("certainty", False):
            self.visualize_lane_certainty(frame, l_perc, r_perc)

        self.prev_lane_coeffs.update({"left": left_coef, "right": right_coef})

        trust_l, trust_r = self.check_lane_certainties(
            l_perc, left_coef, r_perc, right_coef, frame, allowed_difference
        )
        trust_lk = self.trust_lane_keeping(l_perc, r_perc)

        return left_coef, right_coef, l_perc, r_perc, trust_l, trust_r, trust_lk

    def find_lane_peaks(self, slice, height_norm, min_height, min_height_dif, pix_dif, allowed_peaks_width_error):
        max_width = self.peaks_width["max"]
        min_width = self.peaks_width["min"]
        upper_limit = max_width - (max_width - min_width) * height_norm + allowed_peaks_width_error

        slice_data = list(map(int, slice))
        peaks = []
        in_peak = False
        width = 0

        for i in range(pix_dif, len(slice_data) - pix_dif):
            curr = slice_data[i]
            prev = slice_data[i - pix_dif]
            next_val = slice_data[i + pix_dif] if i + pix_dif < len(slice_data) else 0

            if in_peak and (curr <= min_height or next_val - curr > min_height_dif):
                if min_width <= width <= upper_limit:
                    peaks.append(i - (width - pix_dif) // 2)
                in_peak, width = False, 0
            elif not in_peak and curr > min_height and curr - prev > min_height_dif:
                in_peak, width = True, 1
            elif in_peak:
                width += 1

        return peaks
    
    def peaks_clustering(self, points, height, lanes):
        if not lanes:
            return [[[x, height]] for x in points]
        
        lanes_dict = [{"point_index": -1, "distance": -1} for _ in lanes]
        points_dict = [{"used": False, "lane_index": -1} for _ in points]

        run_again = self.find_best_qualified_points(lanes_dict, points_dict, points, lanes, height)
        if run_again:
            self.find_best_qualified_points(lanes_dict, points_dict, points, lanes, height)

        for i, p in enumerate(points_dict):
            if p["used"]:
                lanes[p["lane_index"]].append([points[i], height])
        
        remaining_points = [points[i] for i, p in enumerate(points_dict) if not p["used"]]
        self._insert_remaining_points(remaining_points, lanes, height)
        
        return lanes

    def find_best_qualified_points(self, lanes_dict, points_dict, points, lanes, height):
        run_again = False

        for i, lane in enumerate(lanes):
            if lanes_dict[i]["point_index"] != -1:
                continue

            best_dist, best_idx = float('inf'), -1
            x1, y1 = lane[-1]

            for j, x0 in enumerate(points):
                if points_dict[j]["used"]:
                    continue

                width_dist = abs((x0 - x1) * self.step / (height - y1))
                if width_dist >= self.max_allowed_dist:
                    continue

                flag, width_err = self.verify_with_expected_value(lane, height, x0)
                if not flag:
                    continue

                total_dist = (self.weights["width_distance"] * width_dist + 
                            self.weights["expected_value_distance"] * width_err)
                if total_dist < best_dist:
                    best_dist, best_idx = total_dist, j

            if best_idx != -1:
                prev_idx = points_dict[best_idx]["lane_index"]
                if prev_idx == -1 or lanes_dict[prev_idx]["distance"] > best_dist:
                    if prev_idx != -1:
                        lanes_dict[prev_idx].update({"point_index": -1, "distance": -1})
                        points_dict[best_idx]["used"] = False
                        run_again = True
                    self._add_qualified_point(lanes_dict, points_dict, i, best_idx, best_dist)
                    run_again |= prev_idx != -1

        return run_again and not (all(p["used"] for p in points_dict) or 
                                all(ld["point_index"] != -1 for ld in lanes_dict))

    def _insert_remaining_points(self, remaining_points, lanes, height):
        for point in remaining_points:
            for j, lane in enumerate(lanes):
                if point < lane[-1][0]:
                    lanes.insert(j, [[point, height]])
                    break
            else:
                lanes.append([[point, height]])

    def _add_qualified_point(self, lanes_dict, points_dict, lane_idx, peak_idx, dist):
        lanes_dict[lane_idx].update({"point_index": peak_idx, "distance": dist})
        points_dict[peak_idx].update({"used": True, "lane_index": lane_idx})

    def verify_with_expected_value(self, lane, height, x_value):
        x_dif, y_dif = lane[-1][0] - lane[0][0], lane[-1][1] - lane[0][1]
        x_div_y = x_dif / y_dif if y_dif else 0
        expected = (height - lane[-1][1]) * x_div_y + lane[-1][0]
        min_dist = abs(x_value - expected)
        
        punish = self.max_allowed_dist // 2 if x_div_y == 0 else 0
        punish += ((height - lane[-1][1]) / self.step - 1) / self.real_slices * self.max_allowed_dist

        if len(lane) > 3:
            for k in range(-1, -3, -1):
                xd, yd = lane[k][0] - lane[k-1][0], lane[k][1] - lane[k-1][1]
                slope = xd / yd if yd else 0
                exp = (height - lane[k][1]) * slope + lane[k][0]
                min_dist = min(min_dist, abs(x_value - exp))

        max_dist = self.max_allowed_dist // (2 if x_div_y else 1)
        return (True, min_dist + punish) if min_dist < max_dist else (False, min_dist + punish)

    def calculate_lane_boundaries(self, lane_points):
        x = [lane_points[0][0], lane_points[-1][0]]
        y = [lane_points[0][1], lane_points[-1][1]]

        # Linear fit: x = Slope * y + b
        slope = (x[1] - x[0]) / (y[1] - y[0])
        b = x[0] - slope * y[0]

        # Calculate x-coordinates at top and bottom
        top = slope * self.top_row_index + b
        bot = slope * self.bottom_row_index + b

        return top, bot

    def _fit_polyfit(self, lane, percentage_for_first_degree=0.3):
        if not lane:
            return None
            
        lane_y_x = [[peak[1], peak[0]] for peak in lane]
        
        if len(lane_y_x) > self.slices * percentage_for_first_degree:
            lane_coef = self._fit_polynomial(lane_y_x, degree=2)
        else:
            lane_coef = self._fit_polynomial(lane_y_x, degree=1)
            if lane_coef is not None:
                lane_coef = np.array([0, lane_coef[0], lane_coef[1]])
                
        return lane_coef
        
    def _fit_polynomial(self, points, degree):
        x = np.array([point[0] for point in points])
        y = np.array([point[1] for point in points])
        
        try:
            coef = np.polyfit(x, y, degree)
            limits = {2: self.extreme_coefs["second_deg"], 1: self.extreme_coefs["first_deg"]}
            return coef if math.fabs(coef[0]) < limits[degree] else None
        except (ValueError, np.linalg.LinAlgError):
            return None

    def visualize_lane(self, coefs, frame, bgr_colour=(102, 0, 102)):
        if coefs is None:
            return

        a, b, c = coefs[0], coefs[1], coefs[2]
        
        end_y = int((1 - self.bottom_perc) * self.height)
        y_values = np.arange(end_y, self.height, 3)
        
        x_values = a * y_values**2 + b * y_values + c
        
        points = np.column_stack((x_values.astype(int), y_values))
        
        for i in range(len(points) - 1):
            if (0 <= points[i][0] < self.width and 0 <= points[i+1][0] < self.width):
                cv2.line(frame, tuple(points[i]), tuple(points[i+1]), bgr_colour, thickness=3)

    def find_lane_certainty(self, new_coeffs, prev_coeffs, peaks):
        if prev_coeffs is None or new_coeffs is None:
            return 0.0

        error = np.sqrt(np.mean((new_coeffs - prev_coeffs) ** 2))
        similarity = max(0, 100 - error)

        peaks_percentage = len(peaks) / self.real_slices * 100
        
        certainty = (
            self.perc_params["certainty_from_peaks"] * peaks_percentage
            + (1 - self.perc_params["certainty_from_peaks"]) * similarity
        )

        return round(certainty, 2)

    def trust_lane_keeping(self, l_certainty, r_certainty):
        single_lane_ok = (
            l_certainty > self.min_certainty["single"] or 
            r_certainty > self.min_certainty["single"]
        )
        
        dual_lanes_ok = (
            l_certainty > self.min_certainty["dual"] and 
            r_certainty > self.min_certainty["dual"]
        )
        
        lane_keeping_ok = single_lane_ok or dual_lanes_ok
        
        result = self.prev_trust_lk and lane_keeping_ok
        self.prev_trust_lk = lane_keeping_ok
        
        return result

    def check_lane_certainties(self, left_certainty, left_lane_coeffs, 
                             right_certainty, right_lane_coeffs, frame, 
                             allowed_difference=None):
        allowed_difference = allowed_difference or self.perc_params["allowed_certainty_dif"]
        
        if abs(left_certainty - right_certainty) <= allowed_difference:
            return True, True 

        is_left_trustworthy = left_certainty > right_certainty

        if self.print_flags.get("lanes", False):
            uncertain_lane_coeffs = right_lane_coeffs if is_left_trustworthy else left_lane_coeffs
            self.visualize_lane(uncertain_lane_coeffs, frame, (169, 169, 169))

        return is_left_trustworthy, not is_left_trustworthy

    def visualize_peaks(self, frame, left, right):

        for point in left:
            cv2.circle(frame, (point[0], point[1]), 2, (0, 0, 255), 2)

        for point in right:
            cv2.circle(frame, (point[0], point[1]), 2, (255, 0, 0), 2)

    def visualize_lane_certainty(self, frame, l_perc, r_perc):
        left_text = f"left-{l_perc}"
        right_text = f"right-{r_perc}"
        
        cv2.putText(frame, left_text, (int(0.1 * self.width), int(self.height / 3)), 
                   cv2.FONT_HERSHEY_COMPLEX, 1, (200, 128, 200), 2)
        cv2.putText(frame, right_text, (int(0.6 * self.width), int(self.height / 3)), 
                   cv2.FONT_HERSHEY_COMPLEX, 1, (100, 200, 200), 2)

    def is_dashed(self, lane):

        if not lane or lane[0][1] != self.bottom_row_index:
            return False
            
        lane_index = 0
        inside_dashed_line = True
        point_cnt = 0
        change_state_count = 0
        spaces_count = 0
        length = len(lane)
        
    
        for height in range(self.bottom_row_index, self.top_row_index - 1, self.step):
            if lane_index >= length:
                break
                
            current_height = lane[lane_index][1]
            
            if height == current_height:
                if not inside_dashed_line:
                    # Check if space length is valid
                    if self.min_points_space < point_cnt < self.max_points:
                        inside_dashed_line = True
                        change_state_count += 1
                        point_cnt = 1
                    else:
                        if change_state_count < self.min_count_of_dashed_lanes:
                            return False
                else:
                    point_cnt += 1
                
                spaces_count = 0
                lane_index += 1
                
            elif height > current_height:
                spaces_count += 1
                
                if inside_dashed_line:
                    if spaces_count > self.max_allowed_spaces_inside_a_dashed_lane:
                        if self.min_points < point_cnt < self.max_points:
                            inside_dashed_line = False
                            change_state_count += 1
                            point_cnt = spaces_count
                        else:
                            if change_state_count < self.min_count_of_dashed_lanes:
                                return False
                else:
                    point_cnt += 1
            
        return change_state_count >= self.min_count_of_dashed_lanes
