from BirdBrain.interfaces import DroneAlgorithm, DroneClient
import time


class TestAlgorithm1(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    def main(self):
        self.drone_client.connect()
        for _ in range(20):
            print(self.drone_client.get_altitude())
            time.sleep(1)
        self.drone_client.disconnect()


class TestAlgorithm2(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        self.drone_client.return_to_launch()
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
        
        self.drone_client.return_to_launch()
        self.drone_client.disconnect()


class TestAlgorithm4(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        self.drone_client.goto_target((1,1))

        time.sleep(1)
        
        self.drone_client.return_to_launch()
        self.drone_client.disconnect()


class TestAlgorithm5(DroneAlgorithm):
    def __init__(self, drone_client: DroneClient):
        self.drone_client = drone_client

    def main(self):
        self.drone_client.connect()
        self.drone_client.takeoff()

        self.drone_client.goto_target((1,1))

        time.sleep(2)

        self.drone_client.goto_target((0,1))

        time.sleep(2)
        
        self.drone_client.return_to_launch()
        self.drone_client.disconnect()
