from BirdBrain.interfaces import DroneAlgorithm, DroneClient
from .drone_client import calculate_target_location, get_distance_meters
import time
from dronekit import LocationGlobalRelative


class TestAlgorithm1(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    def main(self):
        self.drone_client.connect()
        for _ in range(10):
            print(self.drone_client.get_altitude())
            time.sleep(1)
        self.drone_client.disconnect()


class TestAlgorithm2(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()
        print("Waiting 5 seconds...")
        time.sleep(5)
        self.drone_client.land()
        time.sleep(5)
        self.drone_client.disconnect()


class TestAlgorithm3(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    def main(self):
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
        self.drone_client = drone_client

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        time.sleep(1)

        self.drone_client.move_forward(1.5)

        time.sleep(2)
        
        self.drone_client.land()
        self.drone_client.disconnect()


class TestAlgorithm5(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    def main(self):
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
        self.drone_client = drone_client

    def main(self):
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
        self.drone_client = drone_client
        
    def wait(self, seconds=3):
        time.sleep(seconds)
        
    def main(self):
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
        self.drone_client = drone_client
        
    def wait(self, seconds=3):
        time.sleep(seconds)
        
    def main(self):
        # Constants
        LONG_FORWARD = 2 
        SHORT_FORWARD = 1 
        TURN_ANGLE = 90
        STEPS = 1  # number of snake segments

        self.drone_client.connect()
        self.drone_client.takeoff()
        self.wait()
        
        # waypoints = [(self.drone_client.get_location(), self.drone_client.get_heading(), "movement")]
        waypoints = [(LocationGlobalRelative(31.76950, 35.19828, self.drone_client.get_initial_altitude()), 90, "movement")]

        for i in range(STEPS):
            # Move long
            waypoints.append((calculate_target_location(
                waypoints[-1][0], waypoints[-1][1], LONG_FORWARD
            ), waypoints[-1][1], "movement"))
            # self.drone_client.move_forward(LONG_FORWARD)
            # self.wait()

            # Turn left (even i) or right (odd i)
            turn = -TURN_ANGLE if i % 2 == 0 else TURN_ANGLE
            waypoints.append((waypoints[-1][0], (waypoints[-1][1] + turn) % 360, "rotation"))
            # self.drone_client.rotate(turn)
            # self.wait()

            # Move short
            # self.drone_client.move_forward(SHORT_FORWARD)
            waypoints.append((calculate_target_location(
                waypoints[-1][0], waypoints[-1][1], SHORT_FORWARD
            ), waypoints[-1][1], "movement"))
            # self.wait()

            # Turn left (even i) or right (odd i)
            # self.drone_client.rotate(turn)
            waypoints.append((waypoints[-1][0], (waypoints[-1][1] + turn) % 360, "rotation"))
            # self.wait()
            
        print(waypoints)
        
        self.drone_client.follow_path(waypoints)

        self.drone_client.land()
        self.drone_client.disconnect()
        
        
        
class TestAlgorithm9(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client
        
    def wait(self, seconds=3):
        time.sleep(seconds)
        
    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()
        self.wait()
        
        waypoints = [(LocationGlobalRelative(31.76950, 35.19828, self.drone_client.get_initial_altitude()), 90, "movement"),
                     (LocationGlobalRelative(31.76950, 35.19828, self.drone_client.get_initial_altitude()), 90, "rotation"),]
        #calc dist from my location to target location
        print(get_distance_meters(
            self.drone_client.get_current_location(), waypoints[0][0]
        ))
        input("Press Enter to continue...")
        input("Press Enter to continue again...")
        self.drone_client.follow_path(waypoints)
        
        self.drone_client.land()
        self.drone_client.disconnect()

        