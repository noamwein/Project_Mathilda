import collections.abc
import time

from BirdBrain.interfaces import DroneAlgorithm, DroneClient, Waypoint, MovementAction
from Rogatka.servo_motor import ServoMotor
from .drone_algorithm import MainDroneAlgorithm

collections.MutableMapping = collections.abc.MutableMapping

from EagleEye.image_detection_models.color_detection_model import ColorImageDetectionModel
from EagleEye.sources.picamera_source import PiCameraSource
from Monitor.gui import MonitorGUI
from Monitor.video_saver import MP4VideoSaver
import threading
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import LocationGlobalRelative
from BirdBrain.settings import START_LAT, START_LON, INITIAL_ANGLE


class TestAlgorithm1(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        for _ in range(10):
            print(self.drone_client.get_altitude())
            time.sleep(1)
        self.drone_client.disconnect()


class TestAlgorithm2(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()
        print("Waiting 5 seconds...")
        time.sleep(5)
        self.drone_client.land()
        time.sleep(5)
        self.drone_client.disconnect()


class TestAlgorithm3(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        self.drone_client.rotate(90)

        time.sleep(1)

        self.drone_client.rotate(-45)

        time.sleep(1)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm4(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        time.sleep(1)

        self.drone_client.move_forward(1.5)

        time.sleep(2)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm5(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        time.sleep(1)

        for i in range(4):
            self.drone_client.move_forward(1.5)

            time.sleep(2)

            self.drone_client.rotate(-90)

            time.sleep(2)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm6(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        time.sleep(3)

        self.drone_client.change_altitude(0.5)

        time.sleep(3)

        self.drone_client.change_altitude(-0.5)

        time.sleep(3)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm7(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def wait(self, seconds=3):
        time.sleep(seconds)

    def _main(self):
        # Constants
        LONG_FORWARD = 3
        SHORT_FORWARD = 1
        TURN_ANGLE = 90
        STEPS = 3  # number of snake segments

        self.drone_client.connect()
        self.drone_client.takeoff()
        self.wait()

        for i in range(STEPS):
            # Move long
            self.drone_client.move_forward(LONG_FORWARD)
            self.wait()

            # Turn left (even i) or right (odd i)
            turn = -TURN_ANGLE if i % 2 == 0 else TURN_ANGLE
            self.drone_client.rotate(turn)
            self.wait()

            # Move short
            self.drone_client.move_forward(SHORT_FORWARD)
            self.wait()

            # Turn left (even i) or right (odd i)
            self.drone_client.rotate(turn)
            self.wait()

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm8(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        self.drone = MainDroneAlgorithm(img_detection=None,
                                        source=None,
                                        drone_client=drone_client,
                                        gui=None,
                                        servo=None)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        waypoints = self.drone.generate_path(steps=0)

        print("Waypoints:", waypoints)

        self.drone_client.follow_path(waypoints, None, None, safe=True, detect=False, stop_on_detect=False)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm9(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        self.drone = MainDroneAlgorithm(img_detection=None,
                                        source=None,
                                        drone_client=drone_client,
                                        servo=None, gui=None)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        # waypoints = [(self.drone_client.get_location(), self.drone_client.get_heading(), "movement")]
        waypoints = self.drone.generate_path(steps=1)

        print("Waypoints:", waypoints)

        self.drone_client.follow_path(waypoints, None, None, safe=True, detect=False, stop_on_detect=False)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm10(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        self.drone = MainDroneAlgorithm(img_detection=None,
                                        source=None,
                                        drone_client=drone_client,
                                        servo=None, gui=None)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        # waypoints = [(self.drone_client.get_location(), self.drone_client.get_heading(), "movement")]
        waypoints = self.drone.generate_path(steps=3)

        print("Waypoints:", waypoints)

        self.drone_client.follow_path(waypoints, None, None, safe=True, detect=False, stop_on_detect=False)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm11(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        # time.sleep(5)

        # self.drone_client.set_speed_no_rotation(0, 0.5, 0)

        time.sleep(5)

        # self.drone_client.set_speed_no_rotation(0, -0.5, 0)

        # time.sleep(5)

        # self.drone_client.set_speed_and_rotate(0.5, 0.5, 0)

        time.sleep(5)

        # self.drone_client.set_speed_and_rotate(-0.5, -0.5, 0)

        time.sleep(5)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm12(DroneAlgorithm):
    '''
    color detection module
    patttern search and log if found
    '''

    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        detection_model = ColorImageDetectionModel(None)
        video_source = PiCameraSource()
        servo = ServoMotor()
        gui = MonitorGUI(drone_client=drone_client, video_saver=MP4VideoSaver(), image_detection=detection_model)
        self.drone = MainDroneAlgorithm(img_detection=detection_model, source=video_source,
                                        drone_client=drone_client, servo=servo, gui=gui)

    def _main(self):
        self.drone._main(only_search=True, stop_on_detect=False)


class TestAlgorithm13(DroneAlgorithm):
    '''
    color detection module
    full operation - pattern search and stop if found
    '''

    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        detection_model = ColorImageDetectionModel(None)
        video_source = PiCameraSource()
        servo = ServoMotor()
        gui = MonitorGUI(drone_client=drone_client, video_saver=MP4VideoSaver(), image_detection=detection_model)
        self.drone = MainDroneAlgorithm(img_detection=detection_model, source=video_source,
                                        drone_client=drone_client, servo=servo, gui=gui)

    def _main(self):
        self.drone._main(only_search=True)


class TestAlgorithm14(DroneAlgorithm):
    '''
    color detection module
    log if target is found
    '''

    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        self.detection_model = ColorImageDetectionModel(None)
        self.video_source = PiCameraSource()

        self.servo = ServoMotor()

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()
        t = time.time()
        while True:
            frame = self.video_source.get_current_frame()
            if self.detection_model.detect_target(frame):
                self.drone_client.log_and_print("Target detected!!!")
        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm15(DroneAlgorithm):
    '''
    color detection module
    rotate to always face target
    '''

    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        detection_model = ColorImageDetectionModel(None)
        video_source = PiCameraSource()
        servo = ServoMotor()
        gui = MonitorGUI(drone_client=drone_client, video_saver=MP4VideoSaver(), image_detection=detection_model)
        self.drone = MainDroneAlgorithm(detection_model, video_source, drone_client, servo, gui)

    def _main(self):
        self.drone._main(search=False, only_rotate=True)


class TestAlgorithm16(DroneAlgorithm):
    '''
    color detection module
    perform full pid
    '''

    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        detection_model = ColorImageDetectionModel(None)
        video_source = PiCameraSource()
        servo = ServoMotor()
        gui = MonitorGUI(drone_client=drone_client, video_saver=MP4VideoSaver(), image_detection=detection_model)
        self.drone = MainDroneAlgorithm(detection_model, video_source, drone_client, servo, gui)

    def _main(self):
        self.drone._main(search=False)


class TestAlgorithm17(DroneAlgorithm):
    '''
    color detection module - full MVP!!
    '''

    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        detection_model = ColorImageDetectionModel(None)
        video_source = PiCameraSource()
        servo = ServoMotor()
        gui = MonitorGUI(drone_client=drone_client, video_saver=MP4VideoSaver(), image_detection=detection_model)
        self.drone = MainDroneAlgorithm(detection_model, video_source, drone_client, servo, gui)

    def _main(self):
        self.drone.main()


class TestAlgorithm18(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):

        waypoints = [Waypoint(position=LocationGlobalRelative(START_LAT, START_LON, 6),
                              angle=INITIAL_ANGLE,
                              movement_action=MovementAction.MOVEMENT)]

        def search_thread():
            self.drone_client.connect()
            self.drone_client.takeoff()

            self.drone_client.follow_path(waypoints, None, None, safe=True, detect=False, stop_on_detect=False)

            self.drone_client.log_and_print("Waiting 100 seconds...")
            time.sleep(100)
            self.drone_client.log_and_print('Finished waiting')

            self.drone_client.land()
            self.drone_client.disconnect()

        thread = threading.Thread(target=search_thread)
        thread.start()

        while thread.is_alive():
            try:
                self.drone_client.log_and_print(f'alt: {self.drone_client.get_altitude()}')
                self.drone_client.log_and_print(f'battery: {self.drone_client.get_battery_voltage()}')
                time.sleep(0.5)
            except:
                pass


class TestAlgorithm19(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        waypoints = [Waypoint(position=self.drone_client.get_current_location(),
                              angle=self.drone_client.get_heading(),
                              movement_action=MovementAction.MOVEMENT)]

        self.drone_client.follow_path(waypoints, None, None, safe=True, detect=False, stop_on_detect=False)

        self.drone_client.log_and_print("Trying set_speed")

        self.drone_client.set_speed(1, 1, 0)

        self.drone_client.log_and_print("Wating 30 seconds...")

        time.sleep(30)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm20(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        waypoints = [Waypoint(position=self.drone_client.get_current_location(),
                              angle=self.drone_client.get_heading(),
                              movement_action=MovementAction.MOVEMENT)]

        self.drone_client.follow_path(waypoints, None, None, safe=True, detect=False, stop_on_detect=False)

        self.drone_client.log_and_print("Trying rotate")

        self.drone_client.rotate(90)

        self.drone_client.log_and_print("Wating 30 seconds...")

        time.sleep(30)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm21(DroneAlgorithm):
    '''
    color detection module
    during manual flight
    '''

    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)
        self.detection_model = ColorImageDetectionModel(None)
        self.video_source = PiCameraSource()
        self.gui = MonitorGUI(drone_client=self.drone_client, video_saver=MP4VideoSaver(),
                              image_detection=self.detection_model)
        self.servo = ServoMotor()

    def _main(self):
        try:
            while True:
                frame = self.video_source.get_current_frame()
                target_position = self.detection_model.locate_target(
                    frame)  # position is in pixels relative to the desired target point
                if self.gui is not None:
                    self.gui.draw_gui(frame)
                if target_position != (None, None):
                    if self.drone_client.is_on_target(target_position):
                        print("boom!")
        except KeyboardInterrupt:
            print("Interrupted by user (Ctrl+C)")
        finally:
            # Always called on exit
            self.gui.video_saver.save_and_close()
            print("Resources released. Exiting cleanly.")


class TestAlgorithm22(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        self.drone_client.set_speed_for_duration(0, -0.5, 0, 5)
        self.drone_client.set_speed_for_duration(0.5, 0, 0, 5)
        self.drone_client.set_speed_for_duration(0, 0.5, 0, 5)
        self.drone_client.set_speed_for_duration(-0.5, 0, 0, 5)

        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm23(DroneAlgorithm):
    '''
    load payload
    '''

    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):
        servo = ServoMotor()
        servo.open_payload()
        time.sleep(1)
        servo.close_payload()


class TestAlgorithm24(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        super().__init__(drone_client)

    def _main(self):

        waypoints = [Waypoint(position=LocationGlobalRelative(START_LAT, START_LON, 6),
                              angle=INITIAL_ANGLE,
                              movement_action=MovementAction.MOVEMENT)]

        def search_thread():
            self.drone_client.connect()
            self.drone_client.takeoff()

            for i in range(100):
                self.drone_client.follow_path(waypoints, None, None, safe=True, detect=False, stop_on_detect=False)

                self.drone_client.log_and_print(f"Waiting, iteration: {i}")
                time.sleep(1)

            self.drone_client.land()
            self.drone_client.disconnect()

        thread = threading.Thread(target=search_thread)
        thread.start()

        while thread.is_alive():
            try:
                self.drone_client.log_and_print(f'alt: {self.drone_client.get_altitude()}')
                self.drone_client.log_and_print(f'battery: {self.drone_client.get_battery_voltage()}')
                time.sleep(0.5)
            except:
                pass