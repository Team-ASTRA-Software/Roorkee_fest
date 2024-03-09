# import utm

# # Assuming lat, lon, alt are latitude, longitude, and altitude
# lat, lon, alt = 37.7749, -122.4194, 0.0
# lat1,lon1,alt= 37.7755,-122.4198,0.0

# # Convert to UTM coordinates
# utm_coords1 = utm.from_latlon(lat, lon)
# utm_coords2 = utm.from_latlon(lat1, lon1)

# # The UTM coordinates are available in utm_coords[0] (easting), utm_coords[1] (northing), utm_coords[2] (zone), utm_coords[3] (zone letter)
# x, y = utm_coords1[0], utm_coords1[1]
# x1,y1=utm_coords2[0], utm_coords2[1]
# print(f"{x} and {y}")
# print(f"{x1} and {y1}")

# import math

# def haversine(lat1, lon1, lat2, lon2):
#     # Convert latitude and longitude from degrees to radians
#     lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

#     # Haversine formula
#     dlat = lat2 - lat1
#     dlon = lon2 - lon1
#     a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
#     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

#     # Radius of Earth in kilometers (change to miles if needed)
#     radius = 6371.0

#     # Calculate the distance
#     distance = radius * c

#     return distance

# def calculate_distance_and_angle(utm1, utm2):
#     lat1, lon1 = utm1
#     lat2, lon2 = utm2

#     # Calculate distance using Haversine formula
#     distance = haversine(lat1, lon1, lat2, lon2)

#     # Calculate angle using arctan2
#     angle = math.atan2(lat2 - lat1, lon2 - lon1)

#     return distance, angle

# # Example UTM coordinates (replace with your values)
# utm_coordinate1 = (x/10000,y/10000)
# utm_coordinate2 = (x/10000,-y/10000)

# distance, angle = calculate_distance_and_angle(utm_coordinate1, utm_coordinate2)

# print(f"Distance: {distance:.2f} km")
# print(f"Angle: {math.degrees(angle):.2f} degrees")
import utm
import math
from geometry_msgs.msg import Quaternion

def calculate_distance_and_angle(utm1, utm2):
    # UTM coordinates are in the format (easting, northing, zone number, zone letter)
    easting1, northing1, _, _ = utm1
    easting2, northing2, _, _ = utm2

    # Calculate distance using Pythagorean theorem
    distance = math.sqrt((easting2 - easting1)**2 + (northing2 - northing1)**2)

    # Calculate angle using arctan2
    angle = math.atan2(northing2 - northing1, easting2 - easting1)

    return distance, angle

# Example UTM coordinates (replace with your values)
utm_coordinate1 = utm.from_latlon( 

47.3978483,8.545679699999999




 )
utm_coordinate2 = utm.from_latlon(47.3977615,8.5456023
 )

distance, angle = calculate_distance_and_angle(utm_coordinate1, utm_coordinate2)

print(f"Distance: {distance:.2f} meters")
print(f"Angle: {(angle):.2f} degrees")



"""
Convert a quaternion into euler angles
taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
"""
q=Quaternion()
x=0.9999043941497803

y=0.007393034640699625


z=0.0003559950564522296


w= 0.011685201898217201





import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


print(euler_from_quaternion(0.7001935243606567

, 0.00028423237381502986
, 0.0008439705707132816


, 0.7139526009559631
))

def degrees_to_radians(angle_degrees:float):
        angle_radians = math.radians(angle_degrees)
        # Ensure the result is in the range from -π to π
        angle_radians = (angle_radians + math.pi) % (2*math.pi) - math.pi
        return angle_radians

print(degrees_to_radians(49.24))

# # - alt: 493.1202087402344
# #   latitude: 47.39774927675293
# #   longitude: 8.545609936196678
# # - alt: 493.11126708984375
# #   latitude: 47.39779371134151
# #   longitude: 8.545604635710545
# # - alt: 493.1399230957031
# #   latitude: 47.39779806134497
# #   longitude: 8.545671128868838