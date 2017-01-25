#!/usr/bin/env python
import time
import json
import rospy
import rospkg
import datetime
import requests


# {
#   "modelId": "LA140",
#   "batteryLevel": 100,
#   "machineStatus": {
#     "systemStatus": "OK",
#     "subsystemsStatuses": [
#       {
#         "componentId": "string",
#         "componentStatus": "OK",
#         "componentDetailedStatus": "OK"
#       }
#     ]
#   },
#   "mapVersion": 1,
#   "isMaintenance": true,
#   "isVirtual": true
# }

# {
#   "batteryLevel": 100,
#   "floorSticker": "1001",
#   "orientation": "90",
#   "currentX": "2400",
#   "currentY": "1200",
#   "mapVersion": 1,
#   "machineStatus": "READY"
# }

def main():
    rospy.init_node("web_test", anonymous=True)
    # path_to_file = rospkg.RosPack().get_path("caja_web") + '/logs'
    # log_time = datetime.datetime.fromtimestamp(time.time()).strftime('%d-%m-%y:%H:%M:%S.%f')
    # log_file = open(path_to_file + '/' + log_time + '-log_web.txt', 'w')
    url_prefix = 'http://52.38.170.95:8080'
    loop_rate = rospy.Rate(10)

    login_request = json.dumps({
        "modelId": "LA120",
        "batteryLevel": 100,
        "machineStatus": {
            "systemStatus": "OK",
            "subsystemsStatuses": [
                {
                    "componentId": "string",
                    "componentStatus": "OK",
                    "componentDetailedStatus": "OK"
                }
            ]
        },
        "mapVersion": 1,
        "isMaintenance": True,
        "isVirtual": True
    })

    keep_alive_request = json.dumps({
        "batteryLevel": 100,
        "floorSticker": "1001",
        "missionInfo": {},
        "orientation": "90",
        "currentX": "2400",
        "currentY": "1200",
        "mapVersion": 1,
        "machineStatus": "READY"
    })

    requests.post(url_prefix + '/machines/999/login', data=login_request, headers={'Content-type': 'application/json'})

    while not rospy.is_shutdown():
        request_time  = rospy.get_rostime()
        requests.post(url_prefix + '/machines/999/keepAlive', data=keep_alive_request, headers={'Content-type': 'application/json'}).json()
        #response_time = rospy.get_rostime().to_nsec()

        dt = (rospy.get_rostime()-request_time).to_sec()

        print "delta: " + str(dt)



        loop_rate.sleep()


    log_file.close()

if __name__ == '__main__':
    main()
