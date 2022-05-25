#!/usr/bin/python2
#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image
#to be able to see only the yellow line and then follow that line
#It uses an approach called proportional and simply means #! /home/weixy/anaconda3/envs/ROS/bin/python



from glob import glob
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time




class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                

                self.image_sub = rospy.Subscriber('camera/image/compressed',
                        CompressedImage, self.image_callback)


                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)
                        
                self.err_pub = rospy.Publisher('chatter', String, queue_size=10)

                self.str = String()

                self.twist = Twist()

        # def autoRight(self):
        #         global coutCor            
        #         self.twist.linear.x = 0.2
        #         self.cmd_vel_pub.publish(self.twist)
        #         rospy.sleep(1)
        #         self.twist.angular.z = -1.5
        #         self.cmd_vel_pub.publish(self.twist)
        #         rospy.sleep(1)







        def image_callback(self, msg):
                global errl,lastErrl, errd, lastErrd, z, lastDetectCor, coutCor, t, start, tl

                kpl = float(1)/50
                kdl = float(1)/80



                np_arr = numpy.fromstring(msg.data,numpy.uint8)#new
                image = cv2.imdecode(np_arr,cv2.IMREAD_COLOR) #new
                # image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                h, w, d = image.shape
                stop = ardetect(image)
                if stop:
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                        rospy.sleep(2)


                lower_yellow = numpy.array([0, 0, 0])
                upper_yellow = numpy.array([180, 255, 70])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # print(mask.shape) (240, 320)
                # thresh = cv2.Canny(mask, 200, 256)
                trace = numpy.zeros(mask.shape,'float32')
                # print(sum(sum(mask)))
      
                # cv2.imshow("mask", mask)


                # for i in range(h):
                #         row = numpy.argwhere(thresh[i,:]>150)
                #         mid = int(sum(row[:]/len(row[:])))
                #         trace[i,mid] = 255
                #         image[i,mid] = [0,0,255]
                
                # H = numpy.array([[-1.87309516e+00, -2.31086212e+00,  4.82588675e+02],[ 5.04927433e-02, -9.15959844e+00,  1.19171381e+03],[ 7.90053744e-05, -1.33380858e-02,  1.00000000e+00]]) # short range
                
                H = numpy.array([[-1.42449518e+00, -2.10743326e+00,  4.04635329e+02],[ 2.69267128e-01, -8.61456334e+00,  1.09721362e+03],[ 4.69093847e-04, -1.23873541e-02,  1.00000000e+00]])# real
                plan_view = cv2.warpPerspective(mask, H, (340, 480))
                # plan_view = numpy.array(plan_view)
                cv2.imshow("plan_view", plan_view)
                rightLow = numpy.zeros(plan_view.shape)
                rightLow[100:-1,170:-1] = plan_view[100:-1,170:-1]
                
                plan_view = rightLow



                cxy = numpy.zeros((3,2),dtype=numpy.float) # 3 areas moments mid points
                errl = numpy.zeros((1,3),dtype=numpy.float) # linear error
                errd = numpy.zeros((1,3),dtype=numpy.float) # degree error
                offset = numpy.array([0.3088549, 0, 0]) # calibrate the midline position
                Ms = [] # moments list
                
                # set the search areas of near,mid,far level
                search_bot_near = 450
                search_top_near = search_bot_near - 20
                search_bot_mid = 350
                search_top_mid = search_bot_mid - 20
                search_bot_far = 250
                search_top_far = search_bot_far - 20
                search_loc = numpy.array([[search_bot_near,search_top_near],[search_bot_mid,search_top_mid],[search_bot_far,search_top_far]],dtype=numpy.int)
                plan_view_r = numpy.zeros((480,340,3))
                for i in range(3):
                        plan_view_r[:,:,i] = plan_view
                # print(plan_view.shape)
                # print(search_loc)
                # cv2.imshow('1',plan_view)
                for i in range(3):
                        maskp = plan_view_r[:,:,i]
                        maskp[0:search_loc[i,1], 0:340] = 0
                        maskp[search_loc[i,0]:480, 0:340] = 0
                        Mp = cv2.moments(maskp)
                        Ms.append(Mp)
                        
                        if Mp['m00'] > 0:
                                cxy[i,0] = Mp['m10']/Mp['m00'] + offset[i]
                                cxy[i,1] = Mp['m01']/Mp['m00']         
                                cv2.circle(plan_view, (int(Mp['m10']/Mp['m00']),int(Mp['m01']/Mp['m00'])) , 10, (0,0,0), -1)
                                errl[0,i] = Mp['m10']/Mp['m00'] - 170 + offset[i]

                # calculate the error angle
                
                if (cxy[1,1]-cxy[0,1]) == 0 :
                        errd[0,0] = 0
                else:
                        errd[0,0] = numpy.rad2deg(- numpy.arctan((cxy[1,0]-cxy[0,0])/((cxy[1,1]-cxy[0,1]))))

                # if (cxy[2,1]-cxy[1,1]) == 0 :
                #         errd[0,1] = 0
                # else:
                #         errd[0,1] = numpy.rad2deg( - numpy.arctan((cxy[2,0]-cxy[1,0])/((cxy[2,1]-cxy[1,1]))))

                # if (cxy[2,1]-cxy[0,1]) == 0:
                #         errd[0,1] = 0
                # else:
                #         errd[0,2] = numpy.rad2deg(- numpy.arctan((cxy[2,0]-cxy[0,0])/((cxy[2,1]-cxy[0,1]))))

                
                # result = cv2.matchTemplate(plan_view.astype(numpy.uint8), cor, cv2.TM_CCOEFF_NORMED)
                # minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(result)
                
                # startX, startY = maxLoc
                # endX = startX + cor.shape[1]
                # endY = startY + cor.shape[0]
               
                # cv2.rectangle(plan_view, (startX, startY), (endX, endY), (255, 0, 0), 3)

                w,h = cor.shape[::-1]
                res = cv2.matchTemplate(plan_view.astype(numpy.uint8),cor,cv2.TM_CCOEFF_NORMED)
                # cv2.imshow('match',res)
                # print(type(res))
                # print(numpy.max(res))


                threshold = 0.8
                loc = numpy.where(res >= threshold)
                for pt in zip(*loc[::-1]):                     
                        if abs(lastDetectCor-time.time()) > 4:
                                coutCor += 1    
                                self.twist.angular.x = 0.15
                                self.twist.angular.z = 0
                                self.cmd_vel_pub.publish(self.twist)
                                rospy.sleep(2.5)
                        cv2.rectangle(plan_view, pt, (pt[0] + w, pt[1] + h), (255,255,255), 2)
                        lastDetectCor = time.time()

                cv2.imshow("rightLow", plan_view)
                cv2.imshow("car view", image)
                cv2.waitKey(3)
                
                if t==0:
                        self.twist.linear.x = 0.2
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                        rospy.sleep(4.5)
                        self.twist.linear.x = 0
                        self.twist.angular.z = -1.5
                        self.cmd_vel_pub.publish(self.twist)
                        rospy.sleep(1.1)
                        self.twist.angular.z = 0
                        # self.twist.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.twist)
                        # rospy.sleep(1)
                        t = time.time()
                        start = 1

                if coutCor == 3 :

                        self.twist.angular.z = 0
                        self.twist.linear.x = 0.1
                        self.cmd_vel_pub.publish(self.twist)
                        rospy.sleep(2.5)
                        self.twist.angular.z = -1.5
                        self.twist.linear.x = 0
                        self.cmd_vel_pub.publish(self.twist)
                        rospy.sleep(1.2)

                        coutCor = 0
                # calculate the differential item

                # derrl = errl - lastErrl
                # derrd = errd - lastErrd


                # lastErrd = errd
                # lastErrl = errl

                # driving strategy
                if Ms[0]['m00'] == 0: # no line detected! searching...
                        # self.twist.linear.x = 0
                        # self.twist.angular.z = 0.5
                        # self.cmd_vel_pub.publish(self.twist)
                        a = []
                else: 

                        
                        # if abs(errl[0,0])<20 and abs(errl[0,1])<20 and abs(errl[0,2])<20:
                        #         self.twist.linear.x = 0.5
                        # else:
                        #         self.twist.linear.x = 0.5
                        # self.twist.angular.z = errd[0,2] * kp

                        # maxderrl = 30
                        # maxderrd = 10

                        # if derrl[0,1]>maxderrl: 
                        #         derrl[0,1] = maxderrl
                        # elif derrl[0,1]<-maxderrl: 
                        #         derrl[0,1] = -maxderrl

                        # if derrd[0,1]>maxderrd: 
                        #         derrd[0,1] = maxderrd
                        # elif derrd[0,1]<-maxderrl: 
                        #         derrd[0,1] = -maxderrd

                        # z = -errl[0,1]*0.03 - derrl[0,1]*0.2 - errd[0,0] * 0.5 - derrd[0,1] * 0.5
                        


                        a = float(1.5)
                        if abs(t-time.time())>a:
                                miderr = (errl[0,1]+errl[0,0])/2 -100
                                # z = -(miderr)*0.02 + errd[0,0]*0.025 #high speed
                                # z = -(miderr)*0.02 - errd[0,0]*0.015
                                # 0.03 0.2 0.5 0.5 z/20  1.1 1   26s 
                                z = -(miderr)*0.015 + errd[0,0]*0.02 #slow
                                # 0.03 0.2 0.3 0.5 30s
                                self.twist.angular.z = z
                                self.twist.linear.x = 0.15
                                print(coutCor)
                                self.cmd_vel_pub.publish(self.twist)
                        else:   
                                z = 0.12
                                x = 0.15
                                self.twist.angular.z = z
                                self.twist.linear.x = x
                                # print(z)
                                self.cmd_vel_pub.publish(self.twist)
                
                # print(miderr)
                # print(z)
                # print(abs(t-time.time()))
                
                
          
                
                # gray = numpy.float32(mask)# transfer to float32
                # dst = cv2.cornerHarris(gray,2,5,0.04)# detect corner
                # dst = cv2.dilate(dst,None) # set image coordinate
                # image[dst>0.6*dst.max()] = [0,0,255] # pin out the corner 
                # ftp = dst>0.6*dst.max() # corner ture false matrix





cor = cv2.imread('/home/weixy/catkin_ws/src/line_follower_turtlebot/scripts/cor.png')

cor = cv2.cvtColor(cor,cv2.COLOR_BGR2GRAY).astype(numpy.uint8)
# print(cor.shape)
# cv2.imshow("car view", cor)
# cv2.waitKey(0)

errl = numpy.zeros((1,3),dtype=numpy.float)
lastErrl = numpy.zeros((1,3),dtype=numpy.float)
errd = numpy.zeros((1,3),dtype=numpy.float)
lastErrd = numpy.zeros((1,3),dtype=numpy.float)
z = float(0)
t = float(0)
tl = float(0)
lastDetectCor = float(0.0)
coutCor = float(0.0)
start = 0



desired_aruco_dictionary = "DICT_6X6_250"          
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250, #set
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
  
def ardetect(frame):
        """
        Main method of the program.
        """


        dist_matrix = numpy.array(([[-0.0 , 0.0, -0.0 , 0.0 ,-0.0]]))
        camera_matrix = numpy.array([[265, 0, 160],[0, 265, 120],[0, 0, 1]])
        # mtx=np.array([[398.12724231  , 0.      ,   304.35638757],
        #  [  0.       ,  345.38259888, 282.49861858],
        #  [  0.,           0.,           1.        ]])

        # Check that we have a valid ArUco marker

        # Load the ArUco dictionary
        # print("[INFO] detecting '{}' markers...".format(
        # desired_aruco_dictionary))
        this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
        this_aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Start the video stream
        #   cap = cv2.VideoCapture(0)

        #   while(True):

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        #     ret, frame = cap.read()  

        # Detect ArUco markers in the video frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
        frame, this_aruco_dictionary, parameters=this_aruco_parameters)
        stop = 0

        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
                # stop = 1
                # Flatten the ArUco IDs list
                ids = ids.flatten()

                # Loop over the detected ArUco corners
                for (marker_corner, marker_id) in zip(corners, ids):
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corner, 0.05, camera_matrix, dist_matrix)
                        distance = ((tvec[0][0][2] + 0.02) * 0.0254) * 100
                        print(distance)
                        if distance <0.8:
                                stop = 1

                        # Extract the marker corners
                        corners = marker_corner.reshape((4, 2))
                        (top_left, top_right, bottom_right, bottom_left) = corners
                                
                        # Convert the (x,y) coordinate pairs to integers
                        top_right = (int(top_right[0]), int(top_right[1]))
                        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                        top_left = (int(top_left[0]), int(top_left[1]))
                                
                        # Draw the bounding box of the ArUco detection
                        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
                                
                        # Calculate and draw the center of the ArUco marker
                        center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                        center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                        cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
                                
                        # Draw the ArUco marker ID on the video frame
                        # The ID is always located at the top_left of the ArUco marker
                        cv2.putText(frame, str(marker_id), 
                                (top_left[0], top_left[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)
                        print('Aruco Marker Detected! ID:  ' + str(marker_id))
                        # Display the resulting frame
                        # cv2.imshow('frame',frame)
                        
        return stop



rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
