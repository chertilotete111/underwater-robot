import time

import cv2
import numpy as np
import math
import pymurapi as mur


def set_motor_power(*args, time_sleep: float = 0.0):
    if time_sleep:
        auv.set_motor_power(*args)
        return time.sleep(time_sleep)
    return auv.set_motor_power(*args)


def set_rgb_color(*args, time_sleep: float = 0.0):
    if time_sleep:
        auv.set_rgb_color(*args)
        return time.sleep(time_sleep)
    return auv.set_rgb_color(*args)


def draw_text(drawing, text, cords=(7, 70)):
    cv2.putText(drawing, str(text), cords, cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)


# статика
class Const:
    # словарь цветов
    COLORS = {
        'orange': ((0, 168, 75),
                   (56, 255, 255)),
        'black': ((49, 255, 0),
                  (61, 255, 35))
    }

    RED_CIRCLE = (
        (101, 125, 57),
        (189, 198, 135)
    )

    TRACKING_LINE_COLOR = (
        (64, 50, 57),
        (255, 121, 120)
    )

    DEPTH = 0.3
    LOW_DEATH = -0.17
    HIGH_DEATH = 1.06
    LINE_COLOR = (100, 150, 255)


class FunctionRunner:
    def __init__(self, function, timeout: float = 0):
        self._function = function
        self._timeout = timeout
        self._prev_timestamp = time.time()

    def run(self):
        timestamp = time.time()
        delta = timestamp - self._prev_timestamp

        if delta > self._timeout:
            self._function()
            self._prev_timestamp = timestamp
            print(self._function)


mur_show = None


class MurDrawer:
    def __init__(self):
        self.prev_frame_time = 0

    def show(self, drawing, camera_id):
        new_frame_time = time.time()
        fps = int(1 / (new_frame_time - self.prev_frame_time))
        self.prev_frame_time = new_frame_time
        fps = str(fps)
        draw_text(drawing, fps, (7, 70))
        mur_show.show(drawing, camera_id)


class PD:
    _kp = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _timestamp = 0

    def __init__(self):
        pass

    def set_p_gain(self, value):
        self._kp = value

    def set_d_gain(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp
        self._prev_error = error
        return output


class Func:
    def __init__(self):
        self._is_enable = True

    def enable(self):
        self._is_enable = True

    def disable(self):
        self._is_enable = False

    def clamp(self, v, max_value, min_value):
        if v > max_value:
            return max_value
        if v < min_value:
            return min_value
        return v

    def clamp_to_180(self, angle: float):
        if angle > 180.0:
            return angle - 360
        if angle < -180.0:
            return angle + 360
        return angle

    def keep_depth(self, depth_to_set: float):
        depth = auv.get_depth()
        error = depth - depth_to_set
        regulator = PD()
        regulator.set_p_gain(150)
        regulator.set_d_gain(70)
        output = regulator.process(error)
        set_motor_power(0, self.clamp(output, 50, -50))
        set_motor_power(3, self.clamp(output, 50, -50))
        print('output depth', output)

    def keep_yaw(self, yaw_to_set, speed):
        regulator = PD()
        regulator.set_p_gain(0.8)
        regulator.set_d_gain(0.5)

        error = auv.get_yaw() - yaw_to_set
        error = self.clamp_to_180(error)
        output = regulator.process(error)
        output = self.clamp(output, 50, -50)
        set_motor_power(0, self.clamp((speed - output), 100, -100))
        set_motor_power(3, self.clamp((speed + output), 100, -100))

    def stabilize_to_x_y(self, x, y):
        x_center = x - (320 / 2)
        y_center = y - (240 / 2)
        length = math.sqrt(x_center ** 2 + y_center ** 2)
        if length < 4.5:
            if context.check_stabilization():
                return True
            else:
                context.reset_stabilization_counter()
        regulator_forward = PD()
        regulator_forward.set_p_gain(0.8)
        regulator_forward.set_d_gain(0.5)

        regulator_side = PD()
        regulator_side.set_p_gain(0.8)
        regulator_side.set_d_gain(0.5)

        output_forward = regulator_forward.process(y_center)
        output_forward = self.clamp(output_forward, -50, 50)
        output_side = regulator_side.process(x_center)
        output_side = self.clamp(output_side, -50, 50)
        set_motor_power(0, output_forward)
        set_motor_power(1, output_forward)
        set_motor_power(3, output_side)
        set_motor_power(2, output_forward)
        # context.set_speed(-output_forward)
        # context.set_side_speed(-output_side)

    def translate_to_90(self):
        yaw = auv.get_yaw()
        if yaw < -180:
            yaw += 90
        if yaw > 180:
            yaw -= 90
        return self.keep_yaw(yaw, 40)

    def translate_to_360(self):
        yaw = auv.get_yaw()
        if yaw < -180:
            yaw += 360
        if yaw > 180:
            yaw -= 360
        return self.keep_yaw(yaw, 40)

    def led_green(self, second):
        auv.set_on_delay(1)
        auv.set_off_delay(0.5)
        set_rgb_color(
            0, 50, 0,
            time_sleep=second
        )

    def led_blue(self, second):
        auv.set_on_delay(1)
        auv.set_off_delay(0.5)
        set_rgb_color(
            0, 0, 50,
            time_sleep=second
        )

    def led_red(self, second):
        auv.set_on_delay(1)
        auv.set_off_delay(0.5)
        set_rgb_color(
            50, 0, 0,
            time_sleep=second
        )

    def color_set(self):
        set_rgb_color(50, 50, 50)


class Missions:
    def __init__(self):
        self.func = Func()
        self.color = ''

    def sorting_mission(
            self,
            shape_name: str,
            color: str,
            x_center: int,
            y_center: int,
            contours,
            drawing
    ):
        if shape_name == 'rectangle' and color in ['black', 'yellow']:
            return
        if shape_name == 'circle' and color in ['yellow', 'black']:
            return
        if shape_name == 'square' and color == 'red':
            return
        if shape_name == 'triangle' and color in ['black', 'red']:
            return
        return self._do_mission(
            shape_name,
            color,
            x_center,
            y_center,
            contours,
            drawing
        )

    def _do_mission(
            self,
            shape_name: str,
            color: str,
            x_center: int,
            y_center: int,
            contours,
            drawing
    ):
        if shape_name != 'rectangle':
            self.func.stabilize_to_x_y(x_center, y_center)

        if shape_name == 'square':
            if color == 'black':
                self._black_square()
            elif color == 'yellow':
                self._yellow_square()

        elif shape_name == 'rectangle':
            pass

        elif shape_name == 'triangle':
            pass

        elif shape_name == 'circle':
            pass

    def _yellow_triangle(self):
        pass

    def _yellow_square(self):
        pass

    def _black_square(self):
        pass

    def _number_place(self):
        pass


class ImageDetector:
    def __init__(self, front_cap, bottom_cap):
        self._front_cap = front_cap  # передняя
        self._bottom_cap = bottom_cap  # нижняя
        self._mission = Missions()  # класс миссий
        self._drawer = MurDrawer()
        self._is_enable = True
        self._bottom_drawing = None

    def enable(self):
        self._is_enable = True

    def disable(self):
        self._is_enable = False

    def __del__(self):
        self._bottom_cap.release()
        self._front_cap.release()

    def _detect_shape(self, cnt):
        area = cv2.contourArea(cnt)

        if area < 1500:
            return None

        # Описанная окружность
        (circle_x, circle_y), circle_radius = cv2.minEnclosingCircle(cnt)
        circle_area = circle_radius ** 2 * 3.14

        # Описанный прямоугольник (с вращением)
        rectangle = cv2.minAreaRect(cnt)

        # Получим контур описанного прямоугольника
        box = cv2.boxPoints(rectangle)
        box = np.int0(box)

        # Вычислим площадь и соотношение сторон прямоугольника.
        rectangle_area = cv2.contourArea(box)
        rect_w, rect_h = rectangle[1][0], rectangle[1][1]
        aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)

        # Описанный треугольник
        try:
            triangle = cv2.minEnclosingTriangle(cnt)[1]
            triangle = np.int0(triangle)
            triangle_area = cv2.contourArea(triangle)
        except:
            triangle_area = 0

        shapes_areas = {
            'circle': circle_area,
            'rectangle' if aspect_ratio > 1.25 else 'square': rectangle_area,
            'triangle': triangle_area
        }

        diffs = {
            name: abs(area - shapes_areas[name]) for name in shapes_areas
        }
        shape_name = min(diffs, key=diffs.get)

        if shape_name == 'circle':
            cv2.circle(
                self._bottom_drawing,
                (int(circle_x), int(circle_y)),
                int(circle_radius),
                Const.LINE_COLOR,
                2, cv2.LINE_AA
            )

        if shape_name == 'rectangle' or shape_name == 'square':
            cv2.drawContours(
                self._bottom_drawing,
                [box],
                0,
                Const.LINE_COLOR,
                2,
                cv2.LINE_AA
            )

        if shape_name == 'triangle':
            cv2.drawContours(
                self._bottom_drawing,
                [triangle],
                0,
                Const.LINE_COLOR,
                2,
                cv2.LINE_AA
            )
        return shape_name, circle_x, circle_y

    def _find_contours(self, color):
        img_hsv = cv2.cvtColor(self._bottom_drawing, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, color[0], color[1])
        contours, _ = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def find_contours_for_every_color(self):
        result = []
        for color_name in Const.COLORS:
            contours = self._find_contours(Const.COLORS[color_name])
            if contours:
                for contour in contours:
                    result.append((contour, color_name))
        return result

    def detect_shape_from_contours(self, contours_with_color):
        for contour, color_name in contours_with_color:
            result = self._detect_shape(contour)
            shape_name, x_center, y_center = '', 0, 0
            if result:
                shape_name, x_center, y_center = result
            if shape_name:
                self._mission.sorting_mission(
                    shape_name,
                    Const.COLORS[color_name],
                    x_center, y_center,
                    contours_with_color,
                    self._bottom_drawing
                )

    def _detect(self):
        bottom_ret, bottom_img = self._bottom_cap.read()
        bottom_img = cv2.GaussianBlur(bottom_img, (7, 7), 0)
        if not bottom_ret:
            return print('cam error')
        self._bottom_drawing = bottom_img.copy()
        contours = self.find_contours_for_every_color()
        self.detect_shape_from_contours(contours)
        self._drawer.show(self._bottom_drawing, 0)

    def start_detect_loop(self):
        if self._is_enable:
            self._detect()


class RedCircleDetection:
    def __init__(self, front_cap, bottom_cap):
        self._front_cap = front_cap
        self._bottom_cap = bottom_cap
        self._is_enable = True
        self._bottom_drawing = None
        self._drawer = MurDrawer()

    def __del__(self):
        self._bottom_cap.release()
        self._front_cap.release()

    def enable(self):
        self._is_enable = True

    def disable(self):
        self._is_enable = False

    def _detect_shape(self, cnt):
        area = cv2.contourArea(cnt)

        if area < 3000:
            return None

        # Описанная окружность
        (circle_x, circle_y), circle_radius = cv2.minEnclosingCircle(cnt)
        circle_area = circle_radius ** 2 * 3.14

        # Описанный прямоугольник (с вращением)
        rectangle = cv2.minAreaRect(cnt)

        # Получим контур описанного прямоугольника
        box = cv2.boxPoints(rectangle)
        box = np.int0(box)

        # Вычислим площадь и соотношение сторон прямоугольника.
        rectangle_area = cv2.contourArea(box)
        rect_w, rect_h = rectangle[1][0], rectangle[1][1]
        aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)

        # Описанный треугольник
        try:
            triangle = cv2.minEnclosingTriangle(cnt)[1]
            triangle = np.int0(triangle)
            triangle_area = cv2.contourArea(triangle)
        except:
            triangle_area = 0

        shapes_areas = {
            'circle': circle_area,
            'rectangle' if aspect_ratio > 1.25 else 'square': rectangle_area,
            'triangle': triangle_area
        }

        diffs = {
            name: abs(area - shapes_areas[name]) for name in shapes_areas
        }
        shape_name = min(diffs, key=diffs.get)

        if shape_name == 'circle':
            cv2.circle(
                self._bottom_drawing,
                (int(circle_x), int(circle_y)),
                int(circle_radius),
                Const.LINE_COLOR,
                2, cv2.LINE_AA
            )

        return shape_name, area, circle_x, circle_y

    def _find_contours(self):
        img_hsv = cv2.cvtColor(self._bottom_drawing, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, Const.RED_CIRCLE[0], Const.RED_CIRCLE[1])
        contours, _ = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def _detect_shape_from_contours(self, contours):
        for contour in contours:
            result = self._detect_shape(contour)
            shape_name, area, x, y = None, None, None, None
            if result:
                shape_name, area, x, y = result
            else:
                return
            if shape_name is not 'circle' and area < 30000:
                func.stabilize_to_x_y(x, y)
                set_motor_power(0, -50)
                set_motor_power(3, -50)
                return
            depth = auv.get_depth()
            context.set_depth(depth)
            image_detector.enable()
            line_detector.enable()
            context.enable()

    def _detect(self):
        bottom_ret, bottom_img = self._bottom_cap.read()
        bottom_img = cv2.GaussianBlur(bottom_img, (7, 7), 0)
        if not bottom_ret:
            return print('cam error')
        self._bottom_drawing = bottom_img.copy()
        contours = self._find_contours()
        self._detect_shape_from_contours(contours)
        self._drawer.show(self._bottom_drawing, 0)

    def red_circle_loop(self):
        if self._is_enable:
            self._detect()


class LineDetector:
    def __init__(self, front_cap, bottom_cap):
        self._front_cap = front_cap  # передняя
        self._bottom_cap = bottom_cap  # нижняя
        self._is_enable = True
        self._func = Func()
        self._drawer = MurDrawer()

    def __del__(self):
        self._front_cap.release()
        self._bottom_cap.release()

    def enable(self):
        self._is_enable = True

    def disable(self):
        self._is_enable = False

    def _find_contours(self, img, color):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, color[0], color[1])
        contours, _ = cv2.findContours(img_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        return contours

    def _detect(self):
        bottom_ret, bottom_img = self._bottom_cap.read()
        bottom_img = cv2.GaussianBlur(bottom_img, (27, 27), 0)
        if not bottom_ret:
            return print('cam error')
        bottom_drawing = bottom_img.copy()
        contours = self._find_contours(bottom_drawing, Const.TRACKING_LINE_COLOR)
        if not contours:
            return
        cv2.drawContours(bottom_drawing, contours, -1, (0, 255, 0), 3)

        if bottom_drawing.shape != (480, 640, 3):
            return

        left_top_rectangle_pos = (
            (0, 0),
            (160, 213)
        )
        right_top_rectangle_pos = (
            (480, 0),
            (640, 213)
        )
        left_bottom_rectangle_pos = (
            (0, 213),
            (160, 480)
        )
        right_bottom_rectangle_pos = (
            (480, 213),
            (640, 480)
        )

        color = (0, 0, 255)
        thickness = 2
        cv2.rectangle(
            bottom_drawing,
            left_top_rectangle_pos[0],
            left_top_rectangle_pos[1],
            color,
            thickness
        )
        cv2.rectangle(
            bottom_drawing,
            right_top_rectangle_pos[0],
            right_top_rectangle_pos[1],
            color,
            thickness
        )
        cv2.rectangle(
            bottom_drawing,
            left_bottom_rectangle_pos[0],
            left_bottom_rectangle_pos[1],
            color,
            thickness
        )
        cv2.rectangle(
            bottom_drawing,
            right_bottom_rectangle_pos[0],
            right_bottom_rectangle_pos[1],
            color,
            thickness
        )
        points = list(np.vstack(contours).squeeze())
        for point in points:
            cv2.circle(bottom_drawing, tuple(point), 1, (0, 255, 0), 2)
        self._drawer.show(bottom_drawing, 0)
        return points

    def _line_walking(self):
        points = self._detect()
        left_top_rectangle_pos = (
            (0, 0),
            (160, 213)
        )
        right_top_rectangle_pos = (
            (480, 0),
            (640, 213)
        )
        left_bottom_rectangle_pos = (
            (0, 213),
            (160, 480)
        )
        right_bottom_rectangle_pos = (
            (480, 213),
            (640, 480)
        )

        if points is None:
            return

        prev_error = 0
        sum_error = 0
        kp, ki, kd = 0.01, 0.0001, 0.01
        for point in points:
            error = 0
            if (left_top_rectangle_pos[0][0] < point[0] < left_top_rectangle_pos[1][0]
                    and left_top_rectangle_pos[0][1] < point[1] < left_top_rectangle_pos[1][1]):
                error = point[0] - left_top_rectangle_pos[0][0]
            elif (right_top_rectangle_pos[0][0] < point[0] < right_top_rectangle_pos[1][0]
                  and right_top_rectangle_pos[0][1] < point[1] < right_top_rectangle_pos[1][1]):
                error = point[0] - right_top_rectangle_pos[0][0]
            elif (left_bottom_rectangle_pos[0][0] < point[0] < left_bottom_rectangle_pos[1][0]
                  and left_bottom_rectangle_pos[0][1] < point[1] < left_bottom_rectangle_pos[1][1]):
                error = point[0] - left_bottom_rectangle_pos[0][0]
            elif (right_bottom_rectangle_pos[0][0] < point[0] < right_bottom_rectangle_pos[1][0]
                  and right_bottom_rectangle_pos[0][1] < point[1] < right_bottom_rectangle_pos[1][1]):
                error = point[0] - right_bottom_rectangle_pos[0][0]

            sum_error += error
            motor_power = (kp * error) + (ki * sum_error) + (kd * (error - prev_error))
            motor_power = self._func.clamp(motor_power, 10, -10)
            if error > 0:
                set_motor_power(1, -motor_power)
                set_motor_power(2, motor_power)
                print(1)
            elif error < 0:
                set_motor_power(1, -motor_power)
                set_motor_power(2, motor_power)
                print(2)
            else:
                set_motor_power(1, -10)
                set_motor_power(2, -10)
                print(3)

            prev_error = error

    def start_line_walking_loop(self):
        if self._is_enable:
            self._line_walking()


class Context:
    _yaw = 0.0
    _depth = 0.5
    _speed = 0.0
    _side_speed = 0.0
    _stabilization_counter = 0
    _shape_counter = 0

    def __init__(self):
        self.func = Func()
        self._is_enable = True

    def enable(self):
        self._is_enable = True

    def disable(self):
        self._is_enable = False

    def get_yaw(self):
        return self._yaw

    def get_death(self):
        return self._depth

    def get_speed(self):
        return self._speed

    def set_yaw(self, value: float):
        self._yaw = value

    def set_depth(self, value: float):
        self._depth = value

    def set_speed(self, value: float):
        self._speed = value

    def set_side_speed(self, value: float):
        self._side_speed = value

    def get_stabilization_counter(self):
        return self._stabilization_counter

    def reset_stabilization_counter(self):
        self._stabilization_counter = 0

    def add_stabilization_counter(self):
        self._stabilization_counter += 1

    def check_stabilization(self, timeout=3):
        if self._stabilization_counter > timeout:
            return True
        else:
            self.add_stabilization_counter()
            return False

    def update(self):
        if not self._is_enable:
            return
        self.func.keep_depth(self._depth)


class Main:
    def __init__(self):
        pass

    def start_polling(self):
        red_circle_detection.disable()

        context.set_depth(Const.DEPTH)
        image_detector.disable()

        context_function = FunctionRunner(function=context.update, timeout=0.2)
        image_detector_runner = FunctionRunner(function=image_detector.start_detect_loop, timeout=0.2)
        line_detector_runner = FunctionRunner(function=line_detector.start_line_walking_loop)
        red_circle_runner = FunctionRunner(function=red_circle_detection.red_circle_loop, timeout=0.2)

        while True:
            red_circle_runner.run()
            context_function.run()
            image_detector_runner.run()
            line_detector_runner.run()


if __name__ == '__main__':
    auv = mur.mur_init()
    mur_show = auv.get_videoserver()

    func = Func()
    main = Main()
    context = Context()
    front_cap = cv2.VideoCapture(0)
    bottom_cap = cv2.VideoCapture(1)

    func.color_set()

    image_detector = ImageDetector(front_cap, bottom_cap)
    line_detector = LineDetector(front_cap, bottom_cap)
    red_circle_detection = RedCircleDetection(front_cap, bottom_cap)

    main.start_polling()
