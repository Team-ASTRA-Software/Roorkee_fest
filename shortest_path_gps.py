import networkx as nx
import math


def haversine_distance(coord1, coord2): #Function to find distance between any two GPS points in meters  
    R = 6371  # Radius of the earth in km

    lat1 = math.radians(coord1[0])
    lon1 = math.radians(coord1[1])
    lat2 = math.radians(coord2[0])
    lon2 = math.radians(coord2[1])

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = (math.sin(dlat / 2) * math.sin(dlat / 2) +
         math.cos(lat1) * math.cos(lat2) *
         math.sin(dlon / 2) * math.sin(dlon / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c * 1000  # Distance in meters

    return distance

def shortest_path(start,waypoints): #Function to find the shortest path with given a starting point and a list of waypoints
    path=list()
    path.append(start)
    all_points=waypoints.copy()
    all_points=[start]+all_points
    curr_pos=start
    
    while(len(path)<len(waypoints)+1):
        mind=100000000
        for w1 in all_points:
            
            if w1!=curr_pos:
                dist=haversine_distance(w1, curr_pos)
                if dist<=mind:
                    mind=dist
                    minp=w1
        path.append(minp)
        all_points.remove(curr_pos)
        # print("Itr done",curr_pos , all_points)
        curr_pos=minp
        

    return path          


# Example usage:
start = (12.8374937, 80.1374365)  # Starting GPS Coordinates
# end = (12.8367525, 80.136861)  # Ending GPS Coordinates
waypoints = [(12.8370584, 80.1372932), (12.8369901, 80.1368858),(12.837443,80.136949),(12.837335,80.137838)]  # Coordinates of all the waypoints in between

result = shortest_path(start, waypoints)

print("Shortest Path:", result)

