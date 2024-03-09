# from pymavlink import mavutil
# import keyboard
# import time
# import folium
# import webbrowser
# def show_on_map(latitude,longitude):
#     # BingMapsKey = 'AsW0swSjS9zAq3VvgYSHwQR95GpaaEaGZR-QGgyNXyoFpCmRTHa_SP5I-VlR0hNw'
#     # tileurl = 'http://dev.virtualearth.net/REST/v1/Locations/%7Bpoint%7D?includeEntityTypes={entityTypes}&includeNeighborhood={includeNeighborhood}&include={includeValue}&key={BingMapsKey}'
#     # map_ = folium.Map(location=[lat, lon],  tiles = tileurl, attr='WHAT SHOULD I PUT HERE?')
#     global map_
#     map_ = folium.Map(location=[20,0])
#     for i in range(0,len(longitude)):
#         folium.Marker(location=[latitude[i], longitude[i]]).add_to(map_)
#     map_.save('map.html')
    

# def main():
#     # Connect to the MAVLink stream
#     master = mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600)  # Replace with your connection detail
#     latitude = list()
#     longitude=list()
#     # Open a file for writing GPS coordinates
#     with open('gps_coordinates.txt', 'w') as file:
#         try:
#             while True:
#                 msgA = master.recv_match(type='ALTITUDE')
#                 msg = master.recv_match(type='GLOBAL_POSITION_INT')
#                 if msgA:
#                         altitude_amsl = msgA.altitude_amsl #Altitude from SeaLevel in Feet
#                         altitude_local=msgA.altitude_local
#                         print("Altitude Received")
#                         print(f'Altitude Sea Level: {altitude_amsl} m, Altitude From Home: {altitude_local} m\n')
                        
#                 # #GPS Logging               
                
#                 # Extract GPS coordinates
#                 if msg:
#                     lat = msg.lat / 1e7  # Convert from degrees * 1e7 to degrees
#                     lon = msg.lon / 1e7  # Convert from degrees * 1e7 to degrees
#                     alt = msg.alt / 1e3  # Convert from millimeters to meters
#                     print("GPS Coordinates Received")
#                     if keyboard.is_pressed('b'):
#                         # Write coordinates to the file
#                         file.write(f'Latitude: {lat}, Longitude: {lon}, Altitude: {alt}\n')
#                         print(f'Latitude: {lat}, Longitude: {lon}, Altitude: {alt}\n')
#                         latitude.append(lat)
#                         longitude.append(lon)                        
#                         file.flush()  # Ensure data is written to the file immediately
                        
#                         # time.sleep(2)

#         except KeyboardInterrupt:
#             print("Entered in except")
#             keyboard.is_pressed('a')
#         finally:
#             show_on_map(latitude,longitude)
#             print("Closing connection and file.")
#             master.close()

# if __name__ == "__main__":
#     main()

from pymavlink import mavutil
import keyboard
import time
import folium
import threading

def show_on_map(latitude, longitude):
    # BingMapsKey = 'AsW0swSjS9zAq3VvgYSHwQR95GpaaEaGZR-QGgyNXyoFpCmRTHa_SP5I-VlR0hNw'
    # tileurl = 'http://dev.virtualearth.net/REST/v1/Locations/%7Bpoint%7D?includeEntityTypes={entityTypes}&includeNeighborhood={includeNeighborhood}&include={includeValue}&key={BingMapsKey}'
    # map_ = folium.Map(location=[lat, lon],  tiles = tileurl, attr='WHAT SHOULD I PUT HERE?')
    global map_
    map_ = folium.Map(location=[20, 0])
    for i in range(0, len(longitude)):
        folium.Marker(location=[latitude[i], longitude[i]]).add_to(map_)
    map_.save('map.html')

def gps_log(master,latitude,longitude,event):
    with open('gps_coordinates.txt', 'w') as file:
        
            while True:
                
                msgA = master.recv_match(type='ALTITUDE')

                if msgA:
                        altitude_amsl = msgA.altitude_amsl  # Altitude from SeaLevel in Feet
                        altitude_local = msgA.altitude_local
                        print("Altitude Received")
                        print(f'Altitude Sea Level: {altitude_amsl} m, Altitude From Home: {altitude_local} m\n')

                # Extract GPS coordinates
                msg = master.recv_match(type='GLOBAL_POSITION_INT')
                if msg:
                    lat = msg.lat / 1e7  # Convert from degrees * 1e7 to degrees
                    lon = msg.lon / 1e7  # Convert from degrees * 1e7 to degrees
                    alt = msg.alt / 1e3  # Convert from millimeters to meters
                    print("GPS Coordinates Received")
                    
                    if event.is_set():
                        file.write(f'Latitude: {lat}, Longitude: {lon}, Altitude: {alt}\n')
                        print(f'Latitude: {lat}, Longitude: {lon}, Altitude: {alt}\n')
                        latitude.append(lat)
                        longitude.append(lon)
                        event.clear()
                        print(event)
                        file.flush()  # Ensure data is written to the file immediately

        
def key(event):
    print('Keythread on')
    while True:
        b=input()
        if b =='b':
            event.set()
            print(event)


            
    
def main():
    # Connect to the MAVLink stream
    master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)  # Replace with your connection detail
    latitude = list()
    longitude = list()
    event =threading.Event()
    event.clear()
    try:
            
            t2 = threading.Thread(target=key,args=(event,))
            t3 = threading.Thread(target=gps_log, args=(master,latitude,longitude,event))

            # t1.start()
            t2.start()
            t3.start()

            # t1.join()
            t2.join()
            t3.join()
                                  
        
    except KeyboardInterrupt:
        show_on_map(latitude, longitude)
        print("Closing connection and file.")
        master.close()

    

if __name__ == "__main__":
    main()