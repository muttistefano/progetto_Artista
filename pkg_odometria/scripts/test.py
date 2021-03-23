# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import paramiko
from scp import SCPClient
from logging import FileHandler
from vlogging import VisualRecord
import logging
import rospy
from sklearn.cluster import KMeans


# image = cv2.imread("img.jpg")

# rospy.init_node('plat_odom')

# pub = rospy.Publisher('topic_name', String, queue_size=10)

np.set_printoptions(suppress=True)

with PiCamera() as camera:
    camera.resolution = (640, 480)
    camera.framerate = 24
    time.sleep(2)
    image = np.empty((480 * 640 * 3,), dtype=np.uint8)
    camera.capture(image, 'bgr')
    image = image.reshape((480, 640, 3))

def createSSHClient(server, port, user, password):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(server, port, user, password)
    return client

# ssh = createSSHClient("10.42.0.1", 22, "kolmogorov", "startx1")
ssh = createSSHClient("192.168.43.63", 22, "kolmogorov", "startx1")
scp = SCPClient(ssh.get_transport())

logger = logging.getLogger("visual_logging_example")
fh = FileHandler("demo.html", mode = "w")
# set the logger attributes
logger.setLevel(logging.DEBUG)
logger.addHandler(fh)


start = time.time()


image_hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

blank_image = np.zeros((image.shape[0],image.shape[1],1))
blank_image.fill(255)
 


edges = cv2.Canny(image_hsv[:,:,1], 100, 250)
edges2 = cv2.Canny(image_hsv[:,:,2], 100, 250)

# lines = cv2.HoughLinesP(edges,5,np.pi/180,50)#,400,100)
# print(len(lines))
# i = 0
# for line in lines:
#     for x1,y1,x2,y2 in line:
#         cv2.line(blank_image,(x1,y1),(x2,y2),(0,255,0),5)

arr_avg = np.asarray([])
print(arr_avg)
lines = cv2.HoughLines(edges,1,np.pi/180,60)
print(len(lines))
print(lines)
for pd in lines:
    for rho,theta in pd:
        arr_avg = np.append(arr_avg,abs(rho))            
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(blank_image,(x1,y1),(x2,y2),(0,0,255),2)


print(arr_avg)
print(np.sort(arr_avg))

kmeans = KMeans(n_clusters=2).fit(arr_avg.reshape(-1,1))
print(kmeans.predict(arr_avg.reshape(-1,1)))

# array([4, 4, 4, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 3,
#       3, 3, 3, 3], dtype=int32)

print(kmeans.cluster_centers_)
# ord_lines_ref = np.empty(shape=(1,2))
# ord_lines_in  = np.empty(shape=(1,2))

# for cnt,line in enumerate(lines_ref):
#     if line[0][1] < 2.7:
#         continue
#     ord_lines_ref = np.append(ord_lines_ref,line,axis=0)

# for cnt,line in enumerate(lines_in):
#     if line[0][1] < 2.7:
#         continue
#     ord_lines_in = np.append(ord_lines_in,line,axis=0)

# ord_lines_ref = np.delete(ord_lines_ref, 0, 0)
# ord_lines_in  = np.delete(ord_lines_in , 0, 0)
# kmeans_ref = KMeans(n_clusters=2, random_state=0).fit(ord_lines_ref)
# kmeans_in  = KMeans(n_clusters=2, random_state=0).fit(ord_lines_in )
# batch1_ref = np.mean(ord_lines_ref[kmeans_ref.labels_==1],axis=0)
# batch2_ref = np.mean(ord_lines_ref[kmeans_ref.labels_==0],axis=0)
# batch1_in = np.mean(ord_lines_in[kmeans_in.labels_==1],axis=0)
# batch2_in = np.mean(ord_lines_in[kmeans_in.labels_==0],axis=0)


end = time.time()
print(end - start)

logger.debug(VisualRecord(("Detected edges using sigma"),
# [image_hsv,image_hsv[:,:,1],edges,image], fmt = "png"))
[image,edges,blank_image], fmt = "png"))

scp.put("demo.html",remote_path='/home/kolmogorov/Documents/ROS/artista/remote_logs')
