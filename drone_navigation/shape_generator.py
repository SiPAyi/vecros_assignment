import numpy as np

def generate_love_symbol_waypoints(center_lat, center_lon, alt=10, scale=0.0001):
    """
    Generates 15 waypoints to form a love (heart) symbol.
    Args:
        center_lat (float): Center latitude of the heart.
        center_lon (float): Center longitude of the heart.
        alt (float): Altitude for all waypoints.
        scale (float): Scaling factor for the heart's size.

    Returns:
        list: A list of 15 dictionaries with 'lat', 'lon', and 'alt'.
    """
    # Heart-shaped parametric equations
    t = np.linspace(0, 2 * np.pi, 15)  # 15 points around the heart
    x = 16 * np.sin(t)**3  # x-coordinates
    y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)  # y-coordinates

    # Scale and translate the heart
    x = x * scale + center_lon
    y = y * scale + center_lat

    # Convert to a list of waypoints
    waypoints = [{"lat": float(lat), "lon": float(lon), "alt": alt} for lat, lon in zip(y, x)]
    return waypoints

def generate_circle_waypoints(center_lat, center_lon, alt=10, radius=0.0001, points=15):
    """
    Generates waypoints to form a circle.
    Args:
        center_lat (float): Center latitude of the circle.
        center_lon (float): Center longitude of the circle.
        alt (float): Altitude for all waypoints.
        radius (float): Radius of the circle.
        points (int): Number of waypoints to form the circle.

    Returns:
        list: A list of dictionaries with 'lat', 'lon', and 'alt'.
    """
    t = np.linspace(0, 2 * np.pi, points)
    x = radius * np.cos(t) + center_lon  # x-coordinates (longitude)
    y = radius * np.sin(t) + center_lat  # y-coordinates (latitude)

    waypoints = [{"lat": float(lat), "lon": float(lon), "alt": alt} for lat, lon in zip(y, x)]
    return waypoints


def generate_star_waypoints(center_lat, center_lon, alt=10, inner_radius=0.00005, outer_radius=0.0001, points=5):
    """
    Generates waypoints to form a star.
    Args:
        center_lat (float): Center latitude of the star.
        center_lon (float): Center longitude of the star.
        alt (float): Altitude for all waypoints.
        inner_radius (float): Inner radius of the star.
        outer_radius (float): Outer radius of the star.
        points (int): Number of star tips.

    Returns:
        list: A list of dictionaries with 'lat', 'lon', and 'alt'.
    """
    t = np.linspace(0, 2 * np.pi, points * 2, endpoint=False)
    r = np.empty_like(t)
    r[::2] = outer_radius  # Outer points
    r[1::2] = inner_radius  # Inner points

    x = r * np.cos(t) + center_lon
    y = r * np.sin(t) + center_lat

    waypoints = [{"lat": float(lat), "lon": float(lon), "alt": alt} for lat, lon in zip(y, x)]
    return waypoints


def generate_spiral_waypoints(center_lat, center_lon, alt=10, scale=0.00001, loops=3, points=30):
    """
    Generates waypoints to form a spiral.
    Args:
        center_lat (float): Center latitude of the spiral.
        center_lon (float): Center longitude of the spiral.
        alt (float): Altitude for all waypoints.
        scale (float): Scaling factor for the spiral's size.
        loops (int): Number of loops in the spiral.
        points (int): Total number of waypoints.

    Returns:
        list: A list of dictionaries with 'lat', 'lon', and 'alt'.
    """
    t = np.linspace(0, loops * 2 * np.pi, points)
    r = scale * t
    x = r * np.cos(t) + center_lon
    y = r * np.sin(t) + center_lat

    waypoints = [{"lat": float(lat), "lon": float(lon), "alt": alt} for lat, lon in zip(y, x)]
    return waypoints


def generate_infinity_waypoints(center_lat, center_lon, alt=10, scale=0.0001, points=30):
    """
    Generates waypoints to form an infinity symbol.
    Args:
        center_lat (float): Center latitude of the infinity symbol.
        center_lon (float): Center longitude of the infinity symbol.
        alt (float): Altitude for all waypoints.
        scale (float): Scaling factor for the size of the infinity symbol.
        points (int): Number of waypoints to form the infinity symbol.

    Returns:
        list: A list of dictionaries with 'lat', 'lon', and 'alt'.
    """
    t = np.linspace(0, 2 * np.pi, points)
    x = scale * np.sin(t) + center_lon
    y = scale * np.sin(t) * np.cos(t) + center_lat

    waypoints = [{"lat": float(lat), "lon": float(lon), "alt": alt} for lat, lon in zip(y, x)]
    return waypoints


if __name__ == "__main__":
    from drone_navigation import *
    
    waypoints = generate_star_waypoints(center_lat=-35.3635, center_lon=149.1641)
    
    # Generate waypoints for the heart shape
    # waypoints = generate_love_symbol_waypoints(center_lat=-35.3635, center_lon=149.1641)

    master = connect_to_drone()
    set_mode(master, "GUIDED")

    arm(master)
    takeoff(master, 10)  # Takeoff to 10 meters altitude  

    plot_path(waypoints)
    send_waypoints(master, waypoints)