import collections.abc
import math
import random
import time

import cv2
import numpy as np
from PIL import Image

collections.MutableMapping = collections.abc.MutableMapping
from dronekit import LocationGlobalRelative

import sys
import os
from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.chrome.options import Options

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from Rogatka.drone_algorithm import MainDroneAlgorithm
from BirdBrain.interfaces import MovementAction, Mapper
from typing import List, Optional


MAPS_PATH = os.path.join(os.path.dirname(__file__), os.path.pardir, os.path.pardir, 'maps')


def get_map_path(loc: LocationGlobalRelative):
    filename = f'{loc.lat:.5f}-{loc.lon:.5f}'
    return os.path.join(MAPS_PATH, 'map_' + filename + '.png')


def get_chromedriver():
    try:
        options = Options()
        options.add_argument("--headless")
        options.add_argument("--window-size=1920x1080")

        driver = webdriver.Chrome(options=options)
    except Exception:
        options = Options()
        options.add_argument('--headless')  # Optional
        options.add_argument('--no-sandbox')
        options.add_argument('--disable-dev-shm-usage')

        # Set path to your chromedriver
        service = Service('/usr/bin/chromedriver')

        # Pass the service to Chrome
        driver = webdriver.Chrome(service=service, options=options)
    return driver


class GovMapper(Mapper):
    EARTH_RADIUS = 6378137  # meters
    SCALE = 250 / 750  # 1:1250 → 1 pixel = 1250 meters

    def __init__(self, locations: List[LocationGlobalRelative], zoom=10, basemap=1):
        super().__init__(locations)
        self.zoom = zoom
        self.basemap = basemap

        min_lat = min(loc.lat for loc in locations)
        max_lat = max(loc.lat for loc in locations)
        min_lon = min(loc.lon for loc in locations)
        max_lon = max(loc.lon for loc in locations)
        center_lat = (min_lat + max_lat) / 2
        center_lon = (min_lon + max_lon) / 2
        self.center_loc = LocationGlobalRelative(center_lat, center_lon)

        self.pil_image = None
        self.cv_image = None
        self.tmp_cv_image = None
        self.height, self.width = None, None

        self.scaling_factor = 1
        if not os.path.exists(get_map_path(self.center_loc)):
            print('Downloading map')
            self.download_map()
            print('Finished downloading map')
        self.load_image()
        self.crop_image()
        self.resize_image(5)
        self.draw_lines()
        self.draw_all_stars()

    def download_map(self):
        driver = get_chromedriver()
        url = f"https://www.govmap.gov.il/?c={self.center_loc.lon},{self.center_loc.lat}&z={self.zoom}&b={self.basemap}"
        driver.get(url)

        time.sleep(5)
        os.makedirs(MAPS_PATH, exist_ok=True)
        driver.save_screenshot(get_map_path(self.center_loc))
        driver.quit()

    def load_image(self):
        self.pil_image = Image.open(get_map_path(self.center_loc))
        self.cv_image = np.array(self.pil_image)

    def gps_to_relative_pixel(self, lat, lon):
        lat_diff = lat - self.center_loc.lat
        lon_diff = lon - self.center_loc.lon

        lat_m = lat_diff * (math.pi / 180) * self.EARTH_RADIUS
        lon_m = lon_diff * (math.pi / 180) * self.EARTH_RADIUS * math.cos(self.center_loc.lat * math.pi / 180)

        x = lon_m / self.SCALE
        y = -lat_m / self.SCALE

        # scale and clip x and y
        x, y = int(x * self.scaling_factor), int(y * self.scaling_factor)
        height, width, _ = self.cv_image.shape
        x, y = min(max(x, -width // 2), width // 2 - 1), min(max(y, -height // 2), height // 2 - 1)
        return x, y

    def gps_to_pixel(self, loc):
        rel_x, rel_y = self.gps_to_relative_pixel(loc.lat, loc.lon)
        height, width, _ = self.cv_image.shape
        x, y = rel_x + width // 2, rel_y + height // 2
        return x, y

    def draw_lines(self):
        for i in range(len(self.locations) - 1):
            start_loc, end_loc = self.locations[i], self.locations[i + 1]
            start_pixel, end_pixel = self.gps_to_pixel(start_loc), self.gps_to_pixel(end_loc)
            cv2.line(self.cv_image, start_pixel, end_pixel, (0, 0, 255))

    def draw_star(self, center, size=10, color=(0, 0, 255)):
        x0, y0 = center
        outer_radius = size
        inner_radius = size * 0.5
        points = []

        for i in range(10):
            angle_deg = i * 36 - 90
            angle_rad = math.radians(angle_deg)
            r = outer_radius if i % 2 == 0 else inner_radius
            x = int(x0 + r * math.cos(angle_rad))
            y = int(y0 + r * math.sin(angle_rad))
            points.append([x, y])

        points = np.array([points], dtype=np.int32)
        cv2.fillPoly(self.cv_image, [points], color)

    def resize_image(self, scaling_factor):
        self.scaling_factor *= scaling_factor
        self.cv_image = cv2.resize(self.cv_image, None, fx=scaling_factor, fy=scaling_factor,
                                   interpolation=cv2.INTER_LINEAR)

    def crop_image(self):
        relative_positions = [self.gps_to_relative_pixel(loc.lat, loc.lon) for loc in self.locations]
        pixels_x, pixels_y = zip(*relative_positions)
        height, width, _ = self.cv_image.shape
        min_x, min_y, max_x, max_y = min(pixels_x) + width // 2, min(pixels_y) + height // 2, max(
            pixels_x) + width // 2, max(pixels_y) + height // 2
        center = (max_x + min_x) // 2, (max_y + min_y) // 2
        new_width = np.array([max(max_x - min_x, 80), max(max_y - min_y, 80)]) + 20
        self.cv_image = self.cv_image[center[1] - new_width[1] // 2:center[1] + new_width[1] // 2,
                        center[0] - new_width[0] // 2:center[0] + new_width[0] // 2, ]

    def draw_all_stars(self):
        for loc in self.locations:
            self.draw_star(self.gps_to_pixel(loc))

    def reset_map(self):
        self.tmp_cv_image = self.cv_image.copy()

    def draw_on_map(self, loc: LocationGlobalRelative):
        x, y = self.gps_to_pixel(loc)
        cv2.circle(self.tmp_cv_image, (x, y), 5, (0, 255, 0), -1)

    def show_map(self):
        cv2.imshow("GPS", self.tmp_cv_image)
        cv2.waitKey(1)

    def get_map(self, current_loc: Optional[LocationGlobalRelative] = None):
        self.reset_map()
        if current_loc is not None:
            self.draw_on_map(current_loc)
        return self.tmp_cv_image


def main():
    path = MainDroneAlgorithm(None, None, None, None, None).generate_new_path()
    test_locations = [waypoint.position for waypoint in path if waypoint.movement_action is MovementAction.MOVEMENT]
    mapper = GovMapper(test_locations)
    while True:
        current_location = LocationGlobalRelative(lat=test_locations[0].lat + random.random() * 1e-5,
                                                  lon=test_locations[0].lon + random.random() * 1e-5)
        map = mapper.get_map(current_location)
        cv2.imshow("GPS", map)
        cv2.waitKey(100)


if __name__ == "__main__":
    main()
