import json
import requests
from datetime import datetime
from pprint import pprint
import math 

from pyzbar.pyzbar import decode

import rospy 
from clover import srv 
from sensor_msgs.msg import Image 
from std_srvs.srv import Trigger 

import cv2 
from cv_bridge import CvBridge

rospy.init_node('flight') 
 
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry) 
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger) 

bridge = CvBridge() 

color = rospy.Publisher('test', Image, queue_size=1)
hsv_topic = rospy.Publisher('hsv', Image, queue_size=1)
orig_topic = rospy.Publisher('orig', Image, queue_size=1)

FLIGHT_HEIGHT = 1
FLIGHT_SPEED = 0.7
HSV_RED_FILTER = ((0, 120, 196), (20, 167, 255))

URL = "https://drone-stats.onrender.com/api/logs"

def navigate_wait(x=0, y=0, z=0, yaw=float("nan"), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def take_off():
    print("Taking off")
    navigate(x=0, y=0, z=1.5, speed=0.5, yaw=float("nan"), frame_id='body', auto_arm=True)
    rospy.sleep(7)
    print("Took off")
    start_x = get_telemetry(frame_id='aruco_map').x
    start_y = get_telemetry(frame_id='aruco_map').y
    rospy.sleep(1.5)
    print(f"Start coords: {start_x}, {start_y}")

    return (start_x, start_y)

def unregister():
    hsv_sub.unregister()
    hsv_topic.unregister()
    orig_topic.unregister()

def image_hsv_callback(data):

    img = bridge.imgmsg_to_cv2(data, 'bgr8')[60:180, 80:240]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, HSV_RED_FILTER[0], HSV_RED_FILTER[1])
    hsv_topic.publish(bridge.cv2_to_imgmsg(mask, 'mono8'))
    orig_topic.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

hsv_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_hsv_callback)

def find_fire(data):

    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    img = img[60:180, 80:240]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, HSV_RED_FILTER[0], HSV_RED_FILTER[1])

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 200]

    result = False
    for cnt in contours:
        cv2.drawContours(img, contours, -1, (255,255,0), 1)
        (x, y, w, h) = cv2.boundingRect(cnt)
        cv2.putText(img, "FIRE", (x + w//2 - 20, y + h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        result = True

    color.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    
    return result

def get_fire():
    res_arr = []
    for _ in range(10):
        data = rospy.wait_for_message('main_camera/image_raw_throttled', Image)    
        res_arr.append(find_fire(data))

    if res_arr.count(True)>=6:
        return True
    return False

def get_qr():
    print("Start qr search")
    
    qr = None

    while qr == None:
        data = rospy.wait_for_message('main_camera/image_raw_throttled', Image)    
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        barcodes = decode(cv_image) 
        for barcode in barcodes:
            qr = barcode.data
            print("Got qr")
    return str(qr)[2:-1]

def get_points(qr):
    points = qr.split('), (')
    points = [list(map(float, i.replace('(', '').replace(')', '').split(', '))) for i in points]

    return points

def get_route(points):
    return sorted(points)

def report(data):

    result = {}
    result["logs"] = []

    for point in data:
        curr = {}
        curr["coordinates"] = list(point[0])
        curr["state"] = point[1]
        result["logs"].append(curr)

    curr_time = datetime.now().strftime("%d.%m.%Y %H:%M")

    result["datetime"] = curr_time

    print("FINAL REPORT:")
    pprint(result)

    json_data = json.dumps(result)

    headers = {
        "Content-Type": "application/json",
        "Accept": "application/json"
    }

    response = requests.post(URL, data=json_data, headers=headers)

def flight():

    (start_x, start_y) = take_off()

    navigate_wait(x=start_x, y=start_y, z=0.7, frame_id="aruco_map")

    qr = get_qr()
    points = get_points(qr)
    print(f"Points from QR code: {points}")

    route_points = get_route(points)

    result_report = []

    for i in range(0, len(route_points)):
        point_x, point_y = route_points[i][0], route_points[i][1]
        
        navigate_wait(x = point_x, y = point_y, z=FLIGHT_HEIGHT, speed=FLIGHT_SPEED)
        rospy.sleep(2)
        print(f"Got to point ({point_x}, {point_y})")
        
        print("Scanning for fire")
        rospy.sleep(1)
        result_report.append(((point_x, point_y), "FIRE" if get_fire() else "Good"))
        if result_report[-1][1] == "FIRE":
            print("Fire")
        else:
            print("Good")
        rospy.sleep(1)

    print("Going to start point")
    navigate_wait(x = start_x, y = start_y, z=FLIGHT_HEIGHT)
    print("Got to start point")

    rospy.sleep(2)

    land()

    report(result_report)

    unregister()

def main():
    flight()

if __name__ == '__main__':
    main()

