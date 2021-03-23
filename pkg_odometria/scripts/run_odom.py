#!/usr/bin/env python
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import paramiko
from scp import SCPClient
from logging import FileHandler
from vlogging import VisualRecord
from std_msgs.msg import Float64MultiArray
import logging
import rospy

np.set_printoptions(suppress=True)

def createSSHClient(server, port, user, password):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(server, port, user, password)
    return client

class platform_odom: 
   
    def __init__(self): 
        self.img_d_num = 20

        self.pub = rospy.Publisher('odom_error', Float64MultiArray, queue_size=1)

        self.getParams()
        rospy.init_node('plat_odom')
        if self.img_debug:
            self.debugSetup()

        width  = 640
        heigth = 480

        self.camera = PiCamera()
        self.camera.resolution = (width, heigth)
        self.camera.framerate = 20
        self.rawCapture = PiRGBArray(self.camera, size=(width, heigth))

        time.sleep(0.5)
        self.visualOdom()
   
    def getParams(self):

        self.img_d_num = rospy.get_param("img_d_num","20")
        rospy.loginfo("img_d_num : %s", self.img_d_num)
        print("img_d_num: ",self.img_d_num)

        self.save_to_disk = rospy.get_param("save_to_disk","False")
        rospy.loginfo("save_to_disk : %s", self.save_to_disk)
        print("save_to_disk: ",self.save_to_disk)

        self.img_debug = rospy.get_param("img_debug","False")
        rospy.loginfo("img_debug : %s", self.img_debug)
        print("img_debug: ",self.img_debug)

    def debugSetup(self):
        # ssh = createSSHClient("10.42.0.1", 22, "kolmogorov", "startx1")
        self.ssh = createSSHClient("192.168.43.63", 22, "kolmogorov", "startx1")#wifi
        self.scp = SCPClient(self.ssh.get_transport())

        self.logger = logging.getLogger("visual_logging_example")
        self.fh = FileHandler("demo.html", mode = "w")
        self.logger.setLevel(logging.DEBUG)
        self.logger.addHandler(self.fh)

    def visualOdom(self):
        
        cnt = 0
        list_img = []
        still = True

        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            if rospy.is_shutdown():
                break

            if cnt>=self.img_d_num and (self.img_debug or self.save_to_disk):
                print(cnt,self.img_d_num,self.img_debug)
                print("Debug finished")
                still=False
                break

            cnt+=1
            try:
                start = time.time()
                image = frame.array 
                image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                edges = cv2.Canny(image_hsv[:,:,1], 100, 250)
                lines = cv2.HoughLines(edges,1,np.pi/180,100)
                # print(len(lines))
                rhos   = np.asarray([])
                thetas = np.asarray([])
                for pd in lines:
                    for rho,theta in pd:
                        if rho < 0:
                            # continue
                            rhos   = np.append(rhos,abs(rho))
                            thetas = np.append(thetas,theta - (3.1415))
                        else:
                            rhos   = np.append(rhos,rho)
                            thetas = np.append(thetas,theta)         

            except Exception as e:
                print(e)
                self.rawCapture.truncate(0)
                continue

            if self.img_debug:
                a = np.cos(np.mean(thetas))
                b = np.sin(np.mean(thetas))
                x0 = a*np.mean(rhos)
                y0 = b*np.mean(rhos)
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)

            if self.save_to_disk or self.img_debug:
                list_img.append(image)

            # print(np.mean(rhos),np.mean(thetas))
            msg = Float64MultiArray(data=[np.mean(rhos),np.mean(thetas)])
            self.pub.publish(msg)

            end = time.time()
            self.rawCapture.truncate(0)

            # print(end - start)

        if self.img_debug:
            print("sending debig file")
            self.logger.debug(VisualRecord(("Detected edges using sigma"),
            # [image_hsv,image_hsv[:,:,1],edges,image], fmt = "png"))
            # [list_img[1],list_img[2],list_img[3],list_img[4]], fmt = "png"))
            list_img, fmt = "png"))

            self.scp.put("demo.html",remote_path='/home/kolmogorov/Documents/ROS/artista/remote_logs')
            print("debug sent")

        if self.save_to_disk:
            for cnt,image_name in enumerate(list_img):
                cv2.imwrite("/home/pi/log_img/img"+str(cnt)+".jpg", image_name)

            print("Images saved")


def main():

  try:
    odom = platform_odom()
    print("Initialized odom")

    rospy.spin()


  except rospy.ROSInterruptException:
    print("exception")
    return
  except KeyboardInterrupt:
    return
  finally:
    print("finally")


if __name__ == '__main__':
  main()
