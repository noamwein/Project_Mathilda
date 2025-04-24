from BirdBrain.interfaces import DroneAlgorithm, DroneClient
import time


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
