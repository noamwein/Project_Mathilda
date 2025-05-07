import math

import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import LocationGlobalRelative

def get_distance_meters(location1, location2):
    """
    Returns the ground distance in meters between two LocationGlobalRelative objects.
    """
    lat_diff = location2.lat - location1.lat
    lon_diff = location2.lon - location1.lon
    return math.sqrt((lat_diff * 111320.0) ** 2 + (lon_diff * 111320.0) ** 2)


def calculate_target_location(current_location, heading, distance):
    """
    Approximate target location for small distances (<100 m) using flat-Earth projection.
    """
    # Constants
    METERS_PER_DEGREE = 111319.5  # Approx. meters per one degree latitude

    # Heading to radians
    heading_rad = math.radians(heading)

    # Compute local offsets in meters (dx east, dy north)
    dx = distance * math.sin(heading_rad)
    dy = distance * math.cos(heading_rad)

    # Current position in degrees
    lat = current_location.lat
    lon = current_location.lon

    # Convert meter offsets to degree offsets
    delta_lat = dy / METERS_PER_DEGREE
    # Longitude scales with cos(lat)
    delta_lon = dx / (METERS_PER_DEGREE * math.cos(math.radians(lat)))

    # Apply offsets
    target_lat = lat + delta_lat
    target_lon = lon + delta_lon

    return LocationGlobalRelative(target_lat, target_lon, current_location.alt)  # unchanged altitude



# def calculate_target_location(current_location, heading, distance):
#     """
#     Calculates the target location based on the current location, heading, and distance.
#     """
#     earth_radius = 6378137.0  # Earth's radius in meters

#     # Convert heading and distance to radians
#     heading_rad = math.radians(heading)
#     distance_rad = distance / earth_radius

#     # Current latitude and longitude in radians
#     lat1 = math.radians(current_location.lat)
#     lon1 = math.radians(current_location.lon)

#     # Calculate target latitude and longitude
#     lat2 = math.asin(math.sin(lat1) * math.cos(distance_rad) +
#                      math.cos(lat1) * math.sin(distance_rad) * math.cos(heading_rad))
#     lon2 = lon1 + math.atan2(math.sin(heading_rad) * math.sin(distance_rad) * math.cos(lat1),
#                              math.cos(distance_rad) - math.sin(lat1) * math.sin(lat2))

#     # Convert back to degrees
#     target_lat = math.degrees(lat2)
#     target_lon = math.degrees(lon2)

    # return LocationGlobalRelative(target_lat, target_lon, current_location.alt)
