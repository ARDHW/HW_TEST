import cv2
from collections import deque
from time import sleep, time
import threading

from dronekit import connect, VehicleMode, Vehicle
import math
from copy import deepcopy

from PySide6.QtPositioning import QGeoCoordinate


class CameraScreenConfiguration:
    __slots__ = (
        'radar_show',
        'radar_size',
        'radar_position',
        'attitude_roll_dig_show',
        'attitude_roll_dig_position',
        'attitude_pitch_dig_show',
        'attitude_pitch_dig_position',
        'param_heading_show',
        'param_heading_position'
    )

    def __init__(self,
                 radar_show=None, radar_size=None, radar_position=None,
                 attitude_roll_dig_show=None, attitude_roll_dig_position=None,
                 attitude_pitch_dig_show=None, attitude_pitch_dig_position=None,
                 param_heading_show=None, param_heading_position=None
                 ):
        self.radar_show = radar_show
        self.radar_position = radar_position
        self.radar_size = radar_size
        self.attitude_roll_dig_show = attitude_roll_dig_show
        self.attitude_roll_dig_position = attitude_roll_dig_position
        self.attitude_pitch_dig_show = attitude_pitch_dig_show
        self.attitude_pitch_dig_position = attitude_pitch_dig_position
        self.param_heading_show = param_heading_show
        self.param_heading_position = param_heading_position


class Drone(Vehicle):
     # __slots__ = ('_attitude, _camera_frame, __camera_capture, _camera_thread')

    # class DroneAttitude:
    #     __slots__ = ('roll', 'pitch')
    #     def __init__(self, roll=None, pitch=None):
    #         self.roll = roll
    #         self.pitch = pitch

    class Target:
        __slots__ = ('target_type', 'target_name', 'target_lat', 'target_lon', )

        def __init__(self, target_type, target_name, target_lat, target_lon):
            self.target_type = target_type
            self.target_name = target_name
            self.target_lat = target_lat
            self.target_lon = target_lon

    class CameraFrame:
        __slots__ = ('attitude', 'heading', 'frame')

        def __init__(self, cur_frame, cur_attitude, cur_heading):
            self.attitude = cur_attitude
            self.frame = cur_frame
            self.heading = cur_heading

        def __del__(self):
            del self.attitude
            del self.frame

    def __init__(self, *args):
        super(Drone, self).__init__(*args)
        # self._attitude = self.DroneAttitude(0,0)

        self.target_list = []
        self._camera_frame = deque(maxlen=50)
        self._camera_thread = None
        self._camera_status = False
        self._camera_is_stop = False
        self.__camera_capture = cv2.VideoCapture(0)

    def add_target(self, target_type, target_name, target_lat, target_lon):
        self.target_list.append(self.Target(target_type, target_name, target_lat, target_lon))
        # vehicle.add_target(type=1, lat=50.39581, lon=30.99291)
        # vehicle.test_caclulate_radar()

    def radar_draw_target(self, cur_frame):
        for cur_target in self.target_list:
            target_geo = QGeoCoordinate(cur_target.target_lat, cur_target.target_lon)

            az = self.tmp_cur_geo.azimuthTo(target_geo)
            ds = self.tmp_cur_geo.distanceTo(target_geo)

            if ds >= 10000 and ds <= 25000:
                coef = 15/15000
                cds = ds - 10000
                pix_len = int(round(cds*coef + 75,0))
            elif ds >= 1000 and ds <= 10000:
                coef = 15/9000
                cds = ds - 1000
                pix_len = int(round(cds*coef + 60,0))
            elif ds >= 100 and ds <= 1000:
                coef = 30 / 900
                cds = ds - 100
                pix_len = int(round(cds * coef + 30, 0))
            elif ds <= 100:
                coef = 30 / 100
                cds = ds
                pix_len = int(round(cds * coef, 0))
            else:
                pix_len = 95

            target_heading = cur_frame.heading - int(round(az, 0))

            target_x_dif = int(round(pix_len*math.sin(target_heading/180*math.pi),0))
            target_y_dif = int(round(pix_len*math.cos(target_heading/180*math.pi),0))

            target_x = 100-target_x_dif
            target_y = 100-target_y_dif

            if cur_target == 2:
                cv2.rectangle(cur_frame.frame, (target_x-1, target_y-1), (target_x+1, target_y+1), (0,0, 255), 2)
            else:
                cv2.circle(cur_frame.frame, (target_x, target_y), 2, (0,0, 255), 2)

    def target_draw_on_image(self, cur_frame):
        for cur_target in self.target_list:
            target_geo = QGeoCoordinate(cur_target.target_lat, cur_target.target_lon)

            az = self.tmp_cur_geo.azimuthTo(target_geo)
            ds = self.tmp_cur_geo.distanceTo(target_geo)

            target_heading = cur_frame.heading - int(round(az, 0))
            if target_heading < -180:
                target_heading = 360 + target_heading
            elif target_heading > 180:
                target_heading = target_heading - 360

            hhdeg = 28
            pix_pd = 320 / hhdeg

            if target_heading >= -hhdeg and target_heading <= hhdeg:
                pix_x = int(round(320 - (pix_pd*target_heading),0))
                cv2.circle(cur_frame.frame, (pix_x, 240), 2, (0,0, 255), 2)

    def __camera_frame_query(self):
        while (not self._camera_is_stop):
            # cur_attitude = self.attitude
            start = time()

            cur_attitude = deepcopy(self.attitude)
            cur_heading = self.heading
            self._camera_status, cur_frame = self.__camera_capture.read()

            # print("Heading: " + str(self.heading))
            # print('read frame processed : ', (time() - start) *1000, 'ms')
            self._camera_frame.append(self.CameraFrame(cur_frame, cur_attitude, cur_heading))

        self.__camera_capture.release()

    def set_cur_pos(self, lat, lon):
        self.tmp_cur_lat = lat
        self.tmp_cur_lon = lon
        self.tmp_cur_geo = QGeoCoordinate(self.tmp_cur_lat, self.tmp_cur_lon)

    def camera_screen_initialize(self, screen_configuration):
        self.camera_screen_configuration = screen_configuration

    def camera_start(self):
        print('camera started!')
        self._camera_thread = threading.Thread(target=self.__camera_frame_query, daemon=True, args=())
        self._camera_thread.start()

    def camera_stop(self):
        self._camera_is_stop = True
        sleep(2)
        self._camera_thread.join()

    def camera_image_get(self):
        if len(self._camera_frame) > 0:
            cur_frame = self._camera_frame.popleft()

            # roll = self.attitude.roll
            roll = cur_frame.attitude.roll

            # print("Roll: " + str(roll))

            x_dif = int(round(math.cos(roll)*200,0))
            y_dif = int(round(math.sin(roll)*200,0))

            x_start = 320 - x_dif
            y_start = 240 + y_dif

            x_end = 320 + x_dif
            y_end = 240 - y_dif

            cv2.line(cur_frame.frame, (x_start, y_start), (x_end, y_end), (255,0,0), 4)

            # font = cv2.FONT_HERSHEY_SIMPLEX
            # org = (50, 50)
            # fontScale = 1
            # color = (255, 0, 0)
            # thickness = 2
            #
            # cv2.putText(cur_frame.frame, str(cur_frame.heading), org, font,
            #             fontScale, color, thickness, cv2.LINE_AA)

            cv2.circle(cur_frame.frame, (100,100), 30, (0,150, 0), 2)
            cv2.circle(cur_frame.frame, (100,100), 60, (0,150, 0), 2)
            cv2.circle(cur_frame.frame, (100,100), 75, (0,150, 0), 2)
            cv2.circle(cur_frame.frame, (100,100), 90, (0,150, 0), 2)
            cv2.circle(cur_frame.frame, (100,100), 2, (0,150, 0), 2)

            self.radar_draw_target(cur_frame)

            self.target_draw_on_image(cur_frame)

            if self.camera_screen_configuration.attitude_roll_dig_show:
                font = cv2.FONT_HERSHEY_SIMPLEX
                org = self.camera_screen_configuration.attitude_roll_dig_position
                fontScale = 1
                color = (0, 0, 255)
                thickness = 2

                cv2.putText(cur_frame.frame, str(round(cur_frame.attitude.roll, 4)), org, font,
                            fontScale, color, thickness, cv2.LINE_AA)

            if self.camera_screen_configuration.attitude_pitch_dig_show:
                font = cv2.FONT_HERSHEY_SIMPLEX
                org = self.camera_screen_configuration.attitude_pitch_dig_position
                fontScale = 1
                color = (0, 0, 255)
                thickness = 2

                cv2.putText(cur_frame.frame, str(round(cur_frame.attitude.pitch, 4)), org, font,
                            fontScale, color, thickness, cv2.LINE_AA)

            if self.camera_screen_configuration.param_heading_show:
                font = cv2.FONT_HERSHEY_SIMPLEX
                org = self.camera_screen_configuration.param_heading_position
                fontScale = 1
                color = (0, 0, 255)
                thickness = 2

                cv2.putText(cur_frame.frame, str(round(cur_frame.heading, 4)), org, font,
                            fontScale, color, thickness, cv2.LINE_AA)



            return cur_frame.frame
        else:
            return None

    def camera_frame_get(self):
        print('current buffers : ', len(self._camera_frame))
        if len(self._camera_frame) > 0:
            return self._camera_frame.popleft()
        else:
            return None

    def update_attitude(self, roll, pitch):
        pass
        # self._attitude = self.DroneAttitude(roll, pitch)


vehicle = connect("com7", wait_ready=True, baud=115200, vehicle_class=Drone, rate=20)
# vehicle = connect("com4", wait_ready=True, baud=115200, vehicle_class=Drone)

vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s' % vehicle.version)

vehicle.set_cur_pos(lat=50.39552, lon=31.00439)

vehicle.add_target(target_type=1, target_name="No1", target_lat=50.39581, target_lon=30.99291)
vehicle.add_target(target_type=2, target_name="Air1", target_lat=50.33434, target_lon=30.89969)
vehicle.add_target(target_type=3, target_name="Hotel", target_lat=50.40496, target_lon=31.01362)
vehicle.add_target(target_type=4, target_name="Tree", target_lat=50.39210, target_lon=31.00770)
vehicle.add_target(target_type=5, target_name="Lake", target_lat=50.39575, target_lon=31.00361)


csc = CameraScreenConfiguration(
    attitude_roll_dig_show=True, attitude_roll_dig_position=(400, 50),
    attitude_pitch_dig_show=True, attitude_pitch_dig_position=(400, 80),
    param_heading_show=True, param_heading_position=(400, 110)
)
vehicle.camera_screen_initialize(csc)


# vehicle.test_caclulate_radar()

vehicle.camera_start()

# sleep(5)

while True:
    # vehicle.camera_frame_read()

    cur_image = vehicle.camera_image_get() # numpy array shape (720, 1280, 3)

    if cur_image is not None:
        cv2.imshow('video', cur_image)
        del cur_image

    if cv2.waitKey(1) == 27:
        cv2.destroyAllWindows()
        break

vehicle.camera_stop()


# sleep(30)


# class camCapture:
#     def __init__(self, camID, buffer_size):
#         self.Frame = deque(maxlen=buffer_size)
#         self.status = False
#         self.isstop = False
#         self.capture = cv2.VideoCapture(camID)
#
#
#     def start(self):
#         print('camera started!')
#         t1 = threading.Thread(target=self.queryframe, daemon=True, args=())
#         t1.start()
#
#     def stop(self):
#         self.isstop = True
#         print('camera stopped!')
#
#
#
#     def getframe(self):
#         print('current buffers : ', len(self.Frame))
#         return self.Frame.popleft()
#
#     def queryframe(self):
#         while (not self.isstop):
#             start = time()
#             self.status, tmp = self.capture.read()
#             print('read frame processed : ', (time() - start) *1000, 'ms')
#             self.Frame.append(tmp)
#
#         self.capture.release()
#
# cam = camCapture(camID=0, buffer_size=1000)
# W, H = 640, 480
# cam.capture.set(cv2.CAP_PROP_FRAME_WIDTH, W)
# cam.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
#
#
# # start the reading frame thread
# cam.start()
#
# # filling frames
# # sleep(5)
#
# while True:
#   # frame = cam.getframe() # numpy array shape (720, 1280, 3)
#
#   # cv2.imshow('video',frame)
#   # sleep( 40 / 1000) # mimic the processing time
#
#   if cv2.waitKey(1) == 27:
#         cv2.destroyAllWindows()
#         cam.stop()
#         break
#
