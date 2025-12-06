import pymurapi as mur

import cv2
import numpy as np
import time

# статика
class Const:
    # словарь цветов
    COLORS = {
        'orange': ((0, 134, 40),
                   (208, 246, 158)),
        'yellow': ((0, 109, 76),
                   (51, 255, 120)),
        'black': ((50, 101, 23),
                  (95, 222, 75))
    }

    TRACKING_LINE_COLOR = ((0, 134, 40),
                           (208, 246, 158))
    DEATH = 0.0
    LOW_DEATH = 0.33
    HIGH_DEATH = 0.0
    FLAG_KEEP_DEATH = 1
    LINE_COLOR = (100, 150, 255)


def set_motor_power(*args, time_sleep: float = 0.0):
    if time_sleep:
        auv.set_motor_power(*args)
        time.sleep(time_sleep)
        return
    return auv.set_motor_power(*args)


def set_rgb_color(*args, time_sleep: float = 0.0):
    if time_sleep:
        auv.set_rgb_color(*args)
        time.sleep(time_sleep)
        return
    return auv.set_rgb_color(*args)


def draw_text(drawing, text, cords=(7, 70)):
    cv2.putText(drawing, str(text), cords, cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)


class FunctionRunner:
    def __init__(self, function, timeout):
        self._function = function
        self._timeout = timeout
        self._prev_timestamp = time.time()

    def run(self):
        timestamp = time.time()
        delta = timestamp - self._prev_timestamp

        if delta > self._timeout:
            self._function()
            self._prev_timestamp = timestamp


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


class PID:
    _kp = 0.0
    _ki = 0.0
    _kd = 0.0
    _prev_error = 0.0
    _integral = 0.0
    _timestamp = 1

    def __init__(self):
        pass

    def set_p_gain(self, value):
        self._kp = value

    def set_i_gain(self, value):
        self._ki = value

    def set_d_gain(self, value):
        self._kd = value

    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        dt = timestamp - self._timestamp
        print('dt', dt)
        if not dt:
            dt = 1
        self._integral += error * dt / 1000
        print('integral', self._integral)
        derivative = (error - self._prev_error) / dt * 1000
        print('derivative', derivative)
        output = self._kp * error + self._ki * self._integral + self._kd * derivative
        self._timestamp = timestamp
        self._prev_error = error
        return output


class Func:
    def __init__(self):
        pass

    def clamp(self, v, max_v, min_v):
        if v > max_v:
            return max_v
        if v < min_v:
            return min_v
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
        regulator.set_p_gain(200)
        regulator.set_d_gain(60)
        output = regulator.process(error)
        set_motor_power(0, output)
        set_motor_power(3, output)
        print('output depth', output)

    def keep_yaw(self, yaw_to_set: float, speed: int):
        error = auv.get_yaw() - yaw_to_set
        error = self.clamp_to_180(error)
        print(error)
        regulator = PID()
        regulator.set_p_gain(50)
        regulator.set_i_gain(100)
        regulator.set_d_gain(0.9)
        output = regulator.process(error)
        print(output)
        set_motor_power(1, self.clamp((speed - output), 50, -50))
        set_motor_power(2, self.clamp((speed + output), 50, -50))

    def translate_to_90(self):
        yaw = auv.get_yaw()
        if yaw < -180:
            yaw += 90
        if yaw > 180:
            yaw -= 90
        self.keep_yaw(yaw, 50)

    def stub_on_shape(self, x_center: int, y_center: int):
        regulator_forward = PID()
        regulator_forward.set_p_gain(0.8)
        regulator_forward.set_i_gain(100)
        regulator_forward.set_d_gain(0.5)

        regulator_side = PID()
        regulator_side.set_p_gain(0.8)
        regulator_side.set_i_gain(100)
        regulator_side.set_d_gain(0.5)

        output_forward = regulator_forward.process(y_center)
        output_forward = self.clamp(output_forward, -50, 50)
        output_side = regulator_side.process(x_center)
        output_side = self.clamp(output_side, -50, 50)
        set_motor_power(0, output_forward)
        set_motor_power(1, output_forward)
        set_motor_power(3, output_side)

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

    def start_keep_death_loop(self):
        if Const.FLAG_KEEP_DEATH:
            self.keep_depth(Const.DEATH)


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
            self.func.stub_on_shape(x_center, y_center)

        if shape_name == 'square':
            if color == 'black':
                self._black_square()
            elif color == 'yellow':
                self._yellow_square()

        elif shape_name == 'rectangle':
            self._go_to_line(contours, drawing)

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

    def _go_to_line(self, contours, drawing):
        pass

    def _number_place(self):
        pass


class ImageDetector:
    def __init__(self, front_cap, bottom_cap):
        self.front_cap = front_cap  # передняя
        self.bottom_cap = bottom_cap  # нижняя
        self.mission = Missions()  # класс миссий
        self.drawer = MurDrawer()
        self.is_enable = True

    def enable(self):
        self.is_enable = True

    def disable(self):
        self.is_enable = False

    def __del__(self):
        self.bottom_cap.release()
        self.front_cap.release()

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
                self.bottom_drawing,
                (int(circle_x), int(circle_y)),
                int(circle_radius),
                Const.LINE_COLOR,
                2, cv2.LINE_AA
            )

        if shape_name == 'rectangle' or shape_name == 'square':
            cv2.drawContours(
                self.bottom_drawing,
                [box],
                0,
                Const.LINE_COLOR,
                2,
                cv2.LINE_AA
            )

        if shape_name == 'triangle':
            cv2.drawContours(
                self.bottom_drawing,
                [triangle],
                0,
                Const.LINE_COLOR,
                2,
                cv2.LINE_AA
            )
        return shape_name, circle_x, circle_y

    def _find_contours(self, img: list, color: tuple):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, color[0], color[1])
        contours, _ = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def find_contours_for_every_color(self, drawing):
        result = []
        for color_name in Const.COLORS:
            contours = self._find_contours(drawing, Const.COLORS[color_name])
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
            # if shape_name:
            #     self.mission.sorting_mission(
            #         shape_name,
            #         Const.COLORS[color_name],
            #         x_center, y_center,
            #         contours,
            #         self.bottom_drawing
            #     )

    def _detect(self):
        bottom_ret, bottom_img = self.bottom_cap.read()
        bottom_img = cv2.GaussianBlur(bottom_img, (27, 27), 0)
        if not bottom_ret:
            return print('cam error')
        self.bottom_drawing = bottom_img.copy()

        self.drawer.show(self.bottom_drawing, 0)

    def start_detect_loop(self):
        if self.is_enable:
            self._detect()


class LineDetector:
    def __init__(self, front_cap, bottom_cap):
        self.front_cap = front_cap  # передняя
        self.bottom_cap = bottom_cap  # нижняя
        self.is_enable = True
        self.func = Func()
        self.drawer = MurDrawer()

    def enable(self):
        self.is_enable = True

    def disable(self):
        self.is_enable = False

    def _find_contours(self, img: list, color: tuple):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, color[0], color[1])
        contours, _ = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def _detect(self):
        bottom_ret, bottom_img = self.bottom_cap.read()
        bottom_img = cv2.GaussianBlur(bottom_img, (27, 27), 0)
        if not bottom_ret:
            return print('cam error')
        bottom_drawing = bottom_img.copy()
        contours = self._find_contours(bottom_drawing, Const.TRACKING_LINE_COLOR)

        cv2.drawContours(bottom_drawing, contours, -1, (0, 255, 0), 3)

        assert bottom_drawing.shape == (480, 640, 3)

        left_top_rectangle_pos = (
            (0, 0),
            (160, 213)
        )
        right_top_rectangle_pos = (
            (480, 0),
            (640, 213)
        )

        color = (255, 0, 0)
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

        points = np.vstack(contours).squeeze()
        for point in points:
            cv2.circle(bottom_drawing, tuple(point), 1, (0, 255, 0), 2)
        self.drawer.show(bottom_drawing, 0)
        return points

    def _line_walking(self):
        points = self._detect()
        left_top_rectangle_pos = ((0, 0), (160, 213))
        right_top_rectangle_pos = ((480, 0), (640, 213))
        for x, y in points:
            lst = set()
            if left_top_rectangle_pos[0][0] < x < left_top_rectangle_pos[1][0] and left_top_rectangle_pos[0][1] < y < \
                    left_top_rectangle_pos[1][1]:
                lst.add(1)
            elif right_top_rectangle_pos[0][0] < x < right_top_rectangle_pos[1][0] and right_top_rectangle_pos[0][
                1] < y < right_top_rectangle_pos[1][1]:
                lst.add(2)
            else:
                lst.add(0)
            lst = list(lst)
            if 1 in lst and 2 in lst:
                self.func.translate_to_90()
            elif 1 in lst:
                # set_motor_power(1, -10) # вправо
                set_motor_power(2, 20)
                time.sleep(1)
            elif 2 in lst:
                set_motor_power(1, 20) # влево
                # set_motor_power(2, -10)
                time.sleep(1)
            elif 0 in lst:
                set_motor_power(1, 30) 
                set_motor_power(2, 30)
                time.sleep(1)

    def start_line_walking_loop(self):
        if self.is_enable:
            self._line_walking()


def main():
    global auv, mur_show
    auv = mur.mur_init()
    mur_show = auv.get_videoserver()  # иннициализация видео сервера
    func = Func()  # инициализация класса функций
    func.color_set()  # установка белого цвета
    front_cap = cv2.VideoCapture(0)
    bottom_cap = cv2.VideoCapture(1)

    image_detector = ImageDetector(front_cap, bottom_cap)  # иннициализация класса фотографии
    line_detector = LineDetector(front_cap, bottom_cap)

    image_detector.disable()

    keep_depth_runner = FunctionRunner(function=func.start_keep_death_loop, timeout=0.2)
    image_detector_runner = FunctionRunner(function=image_detector.start_detect_loop, timeout=0.2)
    line_detector_runner = FunctionRunner(function=line_detector.start_line_walking_loop, timeout=0.2)

    while True:
        keep_depth_runner.run()
        image_detector_runner.run()
        line_detector_runner.run()


if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
