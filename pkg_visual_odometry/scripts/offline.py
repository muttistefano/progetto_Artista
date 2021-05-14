#!/usr/bin/env python

import time
import cv2
import os
import sys
import re
import numpy as np
import imutils

np.set_printoptions(suppress=True)

def findIntersection(x1,y1,x2,y2,x3,y3,x4,y4):
    px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) ) 
    py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )
    return [px, py]


def tryint(s):
    try:
        return int(s)
    except ValueError:
        return s
     
def alphanum_key(s):
    return [ tryint(c) for c in re.split('([0-9]+)', s) ][1]


# def load_images_from_folder(folder):
#     images = []
#     for filename in os.listdir(folder):
#         img = cv2.imread(os.path.join(folder,filename))
#         if img is not None:
#             images.append(img)
#     return images

def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        # img = cv2.imread(os.path.join(folder,filename))
        images.append(filename)
    return images

img_ls = load_images_from_folder(sys.argv[1])
img_ls.sort(key=alphanum_key)
# print(img_ls)

def auto_canny(image, sigma=0.66):
	# compute the median of the single channel pixel intensities
	v = np.median(image)
	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(image, lower, upper)
	# return the edged image
	return edged


for filename in img_ls:


    try:
        start = time.time()
        image = cv2.imread(os.path.join(sys.argv[1],filename))
        # image = imutils.rotate_bound(image, -33)
        img1 = image.copy()
        img2 = image.copy()

        # image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # edges1 = cv2.Canny(image_hsv[:,:,2], 50, 100)
        # edges2 = cv2.Canny(image_hsv[:,:,2], 100, 100)
        # edgesa = auto_canny(image)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        smooth = cv2.GaussianBlur(gray, (21, 21), 0)
        division = cv2.divide(gray, smooth, scale=255)
        edges = cv2.Canny(division, 100, 150)

        lines = cv2.HoughLines(edges,1,np.pi/180,120)

        print("len lines : ",len(lines))

        rhosorig    = np.asarray([])
        thetasorig  = np.asarray([]) 

        rhos    = np.asarray([])
        thetas  = np.asarray([]) 
        rhosx   = np.asarray([])
        thetasx = np.asarray([]) 
        rhosy   = np.asarray([])
        thetasy = np.asarray([]) 

        # for ln in lines:
        #     for rho,theta in ln:
        #         a = np.cos(theta)
        #         b = np.sin(theta)
        #         x0 = a*rho
        #         y0 = b*rho
        #         x1 = int(x0 + 600*(-b))
        #         y1 = int(y0 + 600*(a))
        #         x2 = int(x0 - 600*(-b))
        #         y2 = int(y0 - 600*(a))
        #         cv2.line(img1,(x1,y1),(x2,y2),(0,0,255),2)

        # print(lines)
        for pd in lines:
            for rho,theta in pd:
                rhosorig   = np.append(rhosorig,rho)
                thetasorig = np.append(thetasorig,theta)
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
            print("2 lines")
            rhosx    = rhos[:id_max+1]
            thetasx  = thetas[:id_max+1]
            rhosy    = rhos[id_max+1:]
            thetasy  = thetas[id_max+1:]
        else:
            print("1 line")
            rhosy    = rhos
            thetasy  = thetas
        #     if np.mean(thetas) > np.pi/2.0:
        #         rhosy    = rhos[:id_max+1]
        #         thetasy  = thetas[:id_max+1]
        #         rhosx    = -rhos[id_max+1:]
        #         thetasx  =  np.pi - thetas[id_max+1:]
        #     else:
        #         rhosx    = rhos[:id_max+1]
        #         thetasx  = thetas[:id_max+1]
        #         rhosy    = rhos[id_max+1:]
        #         thetasy  = thetas[id_max+1:]
        # else:
        #     print("1 line")
        #     if np.mean(thetas) > np.pi/2.0:
        #         rhosx    = -rhos
        #         thetasx  = np.pi - thetas 
        #     else:
        #         rhosx    = rhos
        #         thetasx  = thetas
        
        
        

        # for i in range(0,len(rhosx)):
            # cv2.circle(img1, (int(rhosx[i]), 0), 5, (255, 255, 255), 5)
        # for i in range(0,len(rhosy)):
            # cv2.circle(img1, (0,int(rhosy[i])), 5, (255, 255, 255), 5)

        if rhosx.size != 0:
            rhox_mean = np.mean(rhosx)
            thetax_mean = np.mean(thetasx)
            x2 = rhox_mean + 470* (-np.sin(thetax_mean))
            cv2.circle(img1, (int(rhox_mean), 0), 5, (255, 255, 255), 5)
            cv2.circle(img1, (int(x2),470 ), 5, (255, 255, 255), 5)
        if rhosy.size != 0:
            rhoy_mean = np.mean(rhosy)
            thetay_mean = np.mean(thetasy)
            y2 = rhoy_mean + 630* (-np.cos(thetay_mean))
            cv2.circle(img1, ( 0,  int(rhoy_mean)), 5, (255, 255, 255), 5)
            cv2.circle(img1, ( 630,int(y2)), 5, (255, 255, 255), 5)

        if rhosx.size != 0 and rhosy.size != 0:
            a,b = findIntersection(int(rhox_mean), 0, int(x2),470 , 0,  int(rhoy_mean) , 630,int(y2))
            
            cv2.circle(img1, ( int(a),int(b)), 5, (255, 255, 255), 5)

        # print("max id: ",id_max)
        # print(rhosorig)
        # print(thetasorig,"\n\n")
        # print(rhos)
        # print(thetas,"\n\n")
        # # print(grad,"\n\n\n\n\n\n")
        # # print("X Data")
        # print(rhosx)
        # print(thetasx,"\n\n")
        # # print("Y Data")
        # print(rhosy)
        # print(thetasy,"\n\n\n\n\n\n\n\n\n\n\n")
        print(rhoy_mean,thetay_mean)

        # cv2.imshow('imagegsv0',image_hsv[:,:,0])
        # cv2.imshow('imagegsv1',image_hsv[:,:,1])
        # cv2.imshow('imagegsv2',image_hsv[:,:,2])
        # cv2.imshow('image1',division)
        cv2.imshow('ed',edges)
        cv2.imshow('sm',smooth)
        cv2.imshow('image1',img1)
        # cv2.imshow('image2',edges2)
        cv2.waitKey(0)


        # print(len(lines))
        # rhos   = np.asarray([])
        # thetas = np.asarray([])
        # for pd in lines:
        #     for rho,theta in pd:
        #         if rho < 0:
        #             # continue
        #             rhos   = np.append(rhos,abs(rho))
        #             thetas = np.append(thetas,theta - (3.1415))
        #         else:
        #             rhos   = np.append(rhos,rho)
        #             thetas = np.append(thetas,theta)      

        
        # a = np.cos(np.mean(thetas))
        # b = np.sin(np.mean(thetas))
        # x0 = a*np.mean(rhos)
        # y0 = b*np.mean(rhos)
        # x1 = int(x0 + 1000*(-b))
        # y1 = int(y0 + 1000*(a))
        # x2 = int(x0 - 1000*(-b))
        # y2 = int(y0 - 1000*(a))
        # cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)
        # cv2.imshow('image',image)
        # cv2.waitKey(0)   

    except Exception as e:
        print(e)
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        continue





