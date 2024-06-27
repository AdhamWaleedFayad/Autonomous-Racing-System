import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node 
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from prius_msgs.msg import Control
class observer(Node):
    def __init__(self):
        super().__init__('observer')
        self.sub = self.create_subscription(Image,'/prius/front_camera/image_raw',self.visionCallback,10)
        self.odomsub = self.create_subscription(Odometry,'/prius/odom',self.odomcallback,10)
        self.pub = self.create_publisher(Control,'/prius/control',10)
        self.bridge = CvBridge()
        self.odom = Odometry()
        #PID
        self.kp = 1
        self.kd = 0.01
        self.ki = 0.005
        self.dt = 0.01
        self.error = 0.0
        self.error_dot = 0.0
        self.error_integral = 0.0
        self.error_previous = 0.0
        self.previous_steering = 0.0
        self.previous_crosstrack = 0.0

        #state
        self.velocity = 0.0
        self.targetvel = 8
    def pidController(self,target):
        #initializations
        throttle = 0.0
        self.error = target - self.velocity
        self.error_dot = (self.error - self.error_previous)/self.dt
        self.error_integral += self.error*self.dt
        throttle = self.kp*self.error + self.ki*self.error_integral + self.kd*self.error_dot
        self.error_previous = self.error
        return throttle
    def odomcallback(self,msg):
         self.odom = msg
         self.velocity = np.sqrt(np.float_power(self.odom.twist.twist.linear.x , 2) + np.float_power(self.odom.twist.twist.linear.y ,2))
    def visionCallback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        blue_image = cv_image[200:480,:,0]
        lower_blue = np.array([50])
        upper_blue = np.array([255])
        mask = cv2.inRange(blue_image,lower_blue,upper_blue)
        notmask = cv2.bitwise_not(mask)
        #result = cv2.bitwise_and(blue_image,blue_image,mask=notmask)
        lines_list =[]
        blur = cv2.GaussianBlur(notmask,(5,5),0)
        edges = cv2.Canny(blur,50,150,apertureSize=3)
        dilatedEdges = cv2.dilate(edges,(5,5),iterations=1) 
        #dilated = cv2.dilate(edges, kernel, iterations=1)
        lines = cv2.HoughLinesP(
            dilatedEdges, # Input edge image
            1, # Distance resolution in pixels
            np.pi/180, # Angle resolution in radians
            threshold=80, # Min number of votes for valid line
            minLineLength=10, # Min allowed length of line
            maxLineGap=20 # Max allowed gap between line for joining them
            )

        rightLine = [(854,480),(854,480)]
        leftLine = [(0,0),(0,0)]
        rightMid = 9999
        leftMid = 0
        #x_1,y_1,x_2,y_2 = lines[0][0]
        #if y_1>y_2:
        #    if x_1> 854/2: rightLine = [(x_1,y_1),(x_2,y_2)]
        #    else: leftLine = [(x_1,y_1),(x_2,y_2)]
        #else:
        #    if x_2> 854/2: rightLine = [(x1,y1),(x_2,y_2)]
        #    else: leftLine = [(x_1,y_1),(x_2,y_2)]
        # Iterate over points
        
        try:
            if len(lines) >  1:
             for points in lines:
             # Extracted points nested in the list
                 x1,y1,x2,y2=points[0]
                 y1 = y1 + 200
                 y2 = y2 + 200
                 lines_list.append([(x1,y1),(x2,y2)])
                 if (x2> 425):
                         if (x2 < rightLine[1][0]): rightLine = [(x1,y1),(x2,y2)] 
                 if (x2< 430):
                         if (x2 > leftLine[1][0]): leftLine = [(x1,y1),(x2,y2)] 
        except: pass
        #if (np.abs(rightLine[1][0] - leftLine[1][0])):
    
        cv2.line(cv_image,rightLine[0],rightLine[1],(0,255,0))
        cv2.line(cv_image,leftLine[0],leftLine[1],(0,255,0))
        try:
            slopeRight = (rightLine[1][1] - rightLine[0][1])/(rightLine[1][0] - rightLine[0][0])
        except: slopeRight = 0
        try:
            slopeLeft = (leftLine[1][1] - leftLine[0][1])/(leftLine[1][0] - leftLine[0][0])
        except: slopeLeft = 0


        systemEq = np.array([[-slopeRight,1],[-slopeLeft,1]])
        c = np.array([rightLine[1][1] - slopeRight * rightLine[1][0],leftLine[1][1]-slopeLeft*leftLine[1][0]])
        #intersection = [427,240]
        try:
            intersection = np.linalg.solve(systemEq,c)
        except: 
             intersection = [427,240]
        leftLineOnly = 0
        rightLineOnly = 0

        headingCenter = [427,400]
        slopeHeading = 0.0
        if (np.abs(slopeLeft - slopeRight) < 0.1 * (slopeLeft)):
            slopeHeading == slopeLeft
            
        elif slopeLeft == 0:
            slopeHeading = slopeRight
            rightLineOnly = 1
        elif slopeRight == 0:
            slopeHeading = slopeLeft
            leftLineOnly = 1
        else:
            try: 
                slopeHeading = (headingCenter[1] - intersection[1])/(headingCenter[0] - intersection[0])
                leftLineOnly = 1
                rightLineOnly = 1
            except: slopeHeading = 99999
        headingAngle = float(90 - (np.arctan2(headingCenter[1] - intersection[1],(headingCenter[0] - intersection[0])) * (180/np.pi))) #FOR STANLEY
        try:
            headingPoint = [int(np.round((200+slopeHeading*intersection[0]-intersection[1])/slopeHeading)),200]
        except: headingPoint = [427,200]
        
        if (not rightLineOnly):
            crosstrackError = -854
        elif(not leftLineOnly):
            crosstrackError = 854
        crosstrackError = float(leftLineOnly*(427-leftLine[1][0]) + rightLineOnly*(427 - rightLine[1][0])) #FOR STANLEY
          
        #cv2.line(edges,headingCenter,headingPoint,(255,255,255))
        #cv2.putText(edges,str(np.round(crosstrackError)),(430,430),cv2.FONT_HERSHEY_SIMPLEX ,1, (0,0,100),2)
        #cv2.putText(edges,str(headingAngle),(430,410),cv2.FONT_HERSHEY_SIMPLEX ,1, (0,0,0),2)
        cv2.imshow("lines",edges)
        cv2.waitKey(5)
        #STANLEY
        headingComponent = 0.1
        k_crossTrack = 1/400
        k_crossComponent = 3
        k_heading = 0.8
        k_steerBrake = 0.7
        k_soften = 0.1 #for softening crosstrack error in
        
        headingComponent = np.clip(headingAngle/30,-1,1) #NEXT TIME: Try scaling the velocity with (headingComponent-1)
        crosstrackComponent = np.clip(k_crossComponent * np.arctan2((k_crossTrack * crosstrackError),self.velocity+k_soften),-1.1,1.1)
        stanleySteering = np.clip(((headingComponent + crosstrackComponent)),-1,1)
        targetVel = 30 *  1/(1 + 4* (np.abs(stanleySteering) + 5*(np.abs(headingComponent)) + 5* (np.abs(headingComponent))**2 + 4* (np.abs(headingComponent)**2)))
        throttle = 0.0
        throttle = self.pidController(targetVel)
        brake = 0.0
        if throttle < 0: 
             brake = throttle
             throttle = 0.0
        command = Control()
        command.throttle = float(throttle)
        command.brake = brake
        command.steer = stanleySteering
        self.previous_crosstrack = crosstrackComponent
        self.previous_steering = stanleySteering
        try: self.get_logger().info('* Heading angle: %f, Heading Component: %f,Crosstrack error: %f, CrossTrack Component:%f, Velocity: %f,stanleySteering: %f,throttle: %f,brakes: %f,Left Line Only:%d,Right Line Only:%d'% (headingAngle,headingComponent,crosstrackError,crosstrackComponent,self.velocity,stanleySteering,throttle,brake,leftLineOnly,rightLineOnly))
        except: pass
        self.pub.publish(command)



 


        '''
            if (y1>y2): 
                if (x1 > 854/2):
                    if (x1 < rightLine[0][0]): rightLine = [(x1,y1),(x2,y2)] 
                if (x1 < 854/2):
                    if (x1 < leftLine[0][0]): leftLine = [(x1,y1),(x2,y2)] 
            if (y2>y1):
                if (x2 > 854/2):
                    if (x2 < rightLine[0][0]): rightLine = [(x1,y1),(x2,y2)] 
                if (x2 < 854/2):
                    if (x2 < leftLine[0][0]): leftLine = [(x1,y1),(x2,y2)] 
        '''

               
            
            
        



def main():
    rclpy.init()
    print('*Hi from redrunner.*')
    node = observer()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
