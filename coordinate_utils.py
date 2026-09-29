import math

# Earth radius in meters
EARTH_RADIUS_M = 6371000.0

def lat_lon_to_local(lat: float, lon: float, 
                     origin_lat: float, origin_lon: float) -> tuple[float, float]:
    """
    Convert lat/lon to local North/East meters relative to an origin point.
    Uses flat-Earth approximation — accurate enough for distances under 1km.
    
    Returns: (north_m, east_m)
    """
    north_m = (lat - origin_lat) * (math.pi / 180.0) * EARTH_RADIUS_M
    east_m = (lon - origin_lon) * (math.pi / 180.0) * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return north_m, east_m


def local_to_lat_lon(north_m: float, east_m: float,
                     origin_lat: float, origin_lon: float) -> tuple[float, float]:
    """
    Convert local North/East meters back to lat/lon.
    Inverse of lat_lon_to_local.
    
    Returns: (lat, lon)
    """
    lat = origin_lat + (north_m / EARTH_RADIUS_M) * (180.0 / math.pi)
    lon = origin_lon + (east_m / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))) * (180.0 / math.pi)
    return lat, lon


def euclidean_distance(p1: tuple, p2: tuple) -> float:
    """Euclidean distance between two (north, east) points in meters."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# # quick sanity check
# origin = (47.5615, -52.7126)
# point = (47.5625, -52.7116)

# n, e = lat_lon_to_local(point[0], point[1], origin[0], origin[1])
# print(f"North: {n:.2f}m, East: {e:.2f}m")  # should be ~111m north, ~77m east roughly

# # round trip check
# lat, lon = local_to_lat_lon(n, e, origin[0], origin[1])
# print(f"Back to lat/lon: {lat:.6f}, {lon:.6f}")  # should match point exactly