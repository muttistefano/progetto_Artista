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

    def findIntersection(self,x1,y1,x2,y2,x3,y3,x4,y4):
        px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) ) 
        py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )
        return [px, py]

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
                img_log = image.copy()
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                smooth = cv2.GaussianBlur(gray, (21, 21), 0)
                division = cv2.divide(gray, smooth, scale=255)
                edges = cv2.Canny(division, 100, 150)

                lines = cv2.HoughLines(edges,1,np.pi/180,120)

                # print("len lines : ",len(lines))

                rhos    = np.asarray([])
                thetas  = np.asarray([]) 
                rhosx   = np.asarray([])
                thetasx = np.asarray([]) 
                rhosy   = np.asarray([])
                thetasy = np.asarray([]) 

                for pd in lines:
                    for rho,theta in pd:
                        rhos       = np.append(rhos,rho)
                        thetas     = np.append(thetas,theta)  

                rhos[np.argwhere(thetas>2.8)]   = - rhos[np.argwhere(thetas>2.8)] 
                thetas[np.argwhere(thetas>2.8)] = thetas[np.argwhere(thetas>2.8)] - np.pi 
        

                idx    = thetas.argsort()
                rhos   = rhos[idx]
                thetas = thetas [idx]
                grad   = np.diff(thetas)
                id_max = np.argmax(grad)
                if grad[id_max] > 0.5:
                    # print("2 lines")
                    rhosx    = rhos[:id_max+1]
                    thetasx  = thetas[:id_max+1]
                    rhosy    = rhos[id_max+1:]
                    thetasy  = thetas[id_max+1:]
                else:
                    # print("1 line")
                    rhosy    = rhos
                    thetasy  = thetas       

                rhox_mean = np.mean(rhosx)
                thetax_mean = np.mean(thetasx)
                rhoy_mean = np.mean(rhosy)
                thetay_mean = np.mean(thetasy)

            except Exception as e:
                print(e)
                self.rawCapture.truncate(0)
                continue

            if self.img_debug:
                if rhosx.size != 0:
                    x2 = rhox_mean + 470* (-np.sin(thetax_mean))
                    cv2.circle(image, (int(rhox_mean), 0), 5, (255, 255, 255), 5)
                    cv2.circle(image, (int(x2),470 ), 5, (255, 255, 255), 5)
                if rhosy.size != 0:
                    y2 = rhoy_mean + 630* (-np.cos(thetay_mean))
                    cv2.circle(image, ( 0,  int(rhoy_mean)), 5, (255, 255, 255), 5)
                    cv2.circle(image, ( 630,int(y2)), 5, (255, 255, 255), 5)
                if rhosx.size != 0 and rhosy.size != 0:
                    a,b = self.findIntersection(int(rhox_mean), 0, int(x2),470 , 0,  int(rhoy_mean) , 630,int(y2))
                    cv2.circle(image, ( int(a),int(b)), 5, (255, 255, 255), 5)

            if self.save_to_disk or self.img_debug:
                list_img.append(img_log)

            msg = Float64MultiArray(data=[rhoy_mean,thetay_mean])
            self.pub.publish(msg)

            end = time.time()
            self.rawCapture.truncate(0)

            # print(end - start)

        if self.img_debug:
            print("sending debig file")
            self.logger.debug(VisualRecord(("Detected edges using sigma"),
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
