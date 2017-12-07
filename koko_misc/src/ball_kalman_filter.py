#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Accel, Vector3, AccelStamped
from geometry_msgs.msg import PointStamped, PoseStamped
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
import tf

g = -9.81 # m/s^2 acceleration due to gravity (only used in the intersection time solver)

# the intersection time solver math is done assuming gravity is -9.8m/s in the z direction. The actual camera has x in the vertical direction. 
gm = np.array([-9.81,0,0])  #np.array([-9.8,0,0]) # acceleration due to gravity in camera frame: used everywhere else. Positive x direction
camera_to_zgravity_rot = np.array([[0,0,1],[0,1,0],[-1,0,0]]) # used because I did math in the wrong coordinate frame
pi = 3.14159
q = 0.001 # dynamics variance
rx = .00005 # measurement variance (currently assumed to be the same for each axis of the sensor)
ry = .00005
rz = .00005
Q = np.array([[q, 0],[0, q]]) # Dynamics Variance
# Individual Measurement variance equations
Rx = np.array([[rx]])
Ry = np.array([[ry]])
Rz = np.array([[rz]])

class BallKalmanFilter():

    # def __init__(self, pub, base_trans, base_rot):
    def __init__(self, pub, listener): #tryna do a thing hold up
        self.pub = pub

        self.t_last = 0
        self.t_now = 0

        # Used to only start publishing once the taret has been hit a few times (a sign of convergence)
        self.hit_counter = 0
        self.hit_threshold = 0

        # All of the states and state variances. These can be initialized better than 0 later.
        self.x_m = np.zeros((2,1))
        self.Px_m = 100*np.identity(2)

        self.y_m = np.zeros((2,1))
        self.Py_m = 100*np.identity(2)

        self.z_m = np.zeros((2,1))
        self.Pz_m = 100*np.identity(2)

        # Defining the target sphere
        self.mcenter =  np.array([0.2473, -0.0913, 0.3532])
        center_ps = PointStamped()
        center_ps.point.x = self.mcenter[0]
        center_ps.point.y = self.mcenter[1]
        center_ps.point.z = self.mcenter[2]
        center_ps.header = Header()
        center_ps.header.frame_id = 'base_link'
        centerx = listener.transformPoint('kinect_link', center_ps)

        self.center = np.array([centerx.point.x,centerx.point.y,centerx.point.z])
        self.wrist_distance = 0.1 # meters ( 9 inches to the center point of the wrist)




        #self.center =  np.array([0.25, 0.0, .2])  # np.array([0,4,0]) # center of the robot's shoulder CAMERA FRAME
        #self.center =  np.array([0.2473, -0.0913, 0.3532]) #ROBOT SHOULDER

        self.radius = 0.5 # radius at which we want to actually catch / search for intersections.
        # (Wouldn't hurt to make this slightly bigger than what the robot can do so we don't prune the edges)

        # Define transformation from camera frame to world frame
        # self.base_trans = base_trans
        # self.base_rot = base_rot

        # Visualizes sphere of correct radius around the shoulder link
        self.marker_pub = rospy.Publisher('shoulder_marker', Marker, queue_size=1)
        self.marker = Marker()
        #self.marker.header.frame_id = 'base_link'
        self.marker.header.frame_id = 'kinect_link'
     
        self.marker.type = 2
        self.marker.action = 0
        self.marker.pose.orientation.w = 1

        t = rospy.Duration(0)
        self.marker.lifetime = t
        self.marker.scale.x = self.radius*2
        self.marker.scale.y = self.radius*2
        self.marker.scale.z = self.radius*2
        self.marker.color.a = 0.2
        self.marker.color.r = 1

        # Define position of sphere
        self.marker.pose.position.x = self.center[0]
        self.marker.pose.position.y = self.center[1]
        self.marker.pose.position.z = self.center[2]


        # Visualizes the ball trajectory
        self.traj_pub = rospy.Publisher('trajectory_marker', Marker, queue_size=1)

    def pub_trajectory(self, points_list):
        traj = Marker()
        #self.marker.header.frame_id = 'base_link'
        traj.header.frame_id = 'kinect_link'
     
        traj.type = 4
        traj.action = 0

        t = rospy.Duration(0)
        traj.lifetime = t
        traj.scale.x = 0.1
        traj.scale.y = 0.1
        traj.scale.z = 0.1
        traj.color.a = 0.2
        traj.color.g = 1

        # Define position of sphere
        # self.traj.pose.position.x = self.center[0]
        # self.traj.pose.position.y = self.center[1]
        # self.traj.pose.position.z = self.center[2]
        traj.points = []
        for p in points_list:
            traj.points.append(p)
        self.traj_pub.publish(traj)


    def intersection_solver(self,t):
        # Gives the point and velocity after t (seconds)
        zm = self.z_m
        xm = self.x_m
        ym = self.y_m

        xf = np.zeros((2,1))
        yf = np.zeros((2,1))
        zf = np.zeros((2,1))


        xf[0] = xm[0]+xm[1]*t+(gm[0]/2)*t*t
        yf[0] = ym[0] + ym[1]*t+(gm[1]/2)*t*t
        zf[0] = zm[0]+zm[1]*t+(gm[2]/2)*t*t
        xf[1] = xm[1]+(gm[0])*t
        yf[1] = ym[1]+(gm[1])*t
        zf[1] = zm[1]+(gm[2])*t
        return np.array([xf,yf,zf]).reshape((3,2))

    def intersection_time_solver(self, listener):
        # Given the center and radius of the robot's catching sphere
        # and the curent estimates for the trajectory, this calculates
        # if and when the nearest time of intersection will occur,
        # returning false otherwise

        # manually rotated because otherwise it's bad. This solver takes gravity in the -z direction. 
        zm = -self.x_m
        xm = self.z_m
        ym = self.y_m

        #print xm

        # Albert and Rachel are defining a new center. -- David: what the hell is going on here???
        # center_ps = PointStamped()
        # center_ps.point.x = self.center[0]
        # center_ps.point.y = self.center[1]
        # center_ps.point.z = self.center[2]
        # center_ps.header = Header()
        # center_ps.header.frame_id = 'base_link'
        # centerx = listener.transformPoint('kinect_link', center_ps)
        # # center[0] += self.base_trans[0]
        # # Visualizes sphere of 0.5m radius around th
        # # center[1] += self.base_trans[1]
        # # center[2] += self.base_trans[2]

        # center = [0.0, 0.0, 0.0]
        # center[0] = centerx.point.x
        # center[1] = centerx.point.y
        # center[2] = centerx.point.z
        # center = camera_to_zgravity_rot.dot(center)

        # David code added to debug, and undo whatever this is: 

        # all of these calculations are done in a frame with center at the camera's origin, with z pointing straight up. 
        center = camera_to_zgravity_rot.dot(self.center)

        radius = self.radius # same
        # break solution into a manageable form for quartic solver: at^4+bt^3+ct^2+dt+e = 0
        a = g**2/4.0
        b = g*zm[1]
        c = (zm[1]**2+g*(zm[0]-center[2])+xm[1]**2+ym[1]**2)
        d = (2*zm[1]*(zm[0]-center[2])+2*xm[1]*(xm[0]-center[0])+2*ym[1]*(ym[0]-center[1]))
        e = (xm[0]-center[0])**2+(ym[0]-center[1])**2+(zm[0]-center[2])**2-radius**2

        D0 = c**2-3*b*d+12*a*e
        D1 = 2*c**3-9*b*c*d+27*b**2*e+27*a*d**2-72*a*c*e
        p = (8*a*c-3*b**2)/(8*a**2)
        q = (b**3-4*a*b*c+8*a**2*d)/(8*a**3)


        if ((D1**2-4*D0**3) < 0):
            return False
        else:
            Q = ((D1+(D1**2-4*D0**3)**.5)/2)**(1.0/3.0)

            if ((-2*p/3+(1/(3*a))*(Q+D0/Q))) <= 0:
                return False
            else:
                S = .5*(-2*p/3+(1/(3*a))*(Q+D0/Q))**.5

        large_number = 4.0000000
        t1 = large_number
        t2 = large_number
        t3 = large_number
        t4 = large_number
        if ((-4*S**2-2*p+q/S) > 0):
            t1 = -b/(4*a)-S+.5*(-4*S**2-2*p+q/S)**.5
            t2 = -b/(4*a)-S-.5*(-4*S**2-2*p+q/S)**.5
            t1 = t1[0]
            t2 = t2[0]

        if ((-4*S**2-2*p-q/S) > 0):
            t3 = -b/(4*a)+S+.5*(-4*S**2-2*p-q/S)**.5
            t4 = -b/(4*a)+S-.5*(-4*S**2-2*p-q/S)**.5
            t3 = t3[0]
            t4 = t4[0]

        # # hackery shinenigans to get the second intersection point (second smallest number that is positive)
        # # get positive numbers
        # pos_list = [n for n in [t1,t2,t3,t4] if n > 0]
        # pos_list = np.flatten(pos_list)
        # print pos_list

        # pos_nums = set(pos_list)
        # if len(pos_nums) > 1:
        #   tfinal = sorted(pos_nums)[1]
        # else:
        #   tfinal = sorted(pos_nums)[0]
        tpos = [n for n in [t1,t2,t3,t4] if n > -.1]
        if len(tpos) == 3:
            take_first = True
        else:
            take_first = False

        try:

            tpossorted = sorted(tpos)
        except:
            import IPython
            IPython.embed()
        print tpossorted
        if take_first is True:
            tfinal = tpossorted[0]
        else:
            tfinal = (tpossorted[0] + tpossorted[1]) / 2.0

        #tfinal = min([n for n in [t1,t2,t3,t4] if n > -.1])#min(t1,t2,t3,t4)
        #print [t1,t2,t3,t4,tfinal]

        if (tfinal == large_number):
            # final check of intersection
            return False

        final_pose = self.intersection_solver(tfinal)
        final_radius = ((final_pose[0,0]-self.center[0])**2+(final_pose[1,0]-self.center[1])**2+(final_pose[2,0]-self.center[2])**2)**(0.5)
        if (abs(final_radius-self.radius) > .05):
            return False
        return tfinal

    def callback(self, message, timestamp, fid, listener):
        # Run a step of the filter when this has recieved a new point, and publish if the hit counter is passed
        # For TF message:
        # x_meas = message.transform.translation.x
        # y_meas = message.transform.translation.y
        # z_meas = message.transform.translation.z

        # measured states
        # x_meas = message.point.x
        # y_meas = message.point.y
        # z_meas = message.point.z

        x_meas = message[0]
        y_meas = message[1]
        z_meas = message[2]

        

        # print z_meas

        #print x_meas

        # t_now is the time when the most recent image was taken
        # self.t_now = (message.header.stamp).to_sec()
        self.t_now = timestamp.to_sec()

        dt = self.t_now-self.t_last
        self.t_last = self.t_now

        # calculate dt: change in time from last measuremet
        if (dt > .1):
            dt = 0
            # if the change in time is greater than some value, reset variances to be big!
            self.hit_counter = 0
            self.Px_m = 100*np.identity(2)
            self.Py_m = 100*np.identity(2)
            self.Pz_m = 100*np.identity(2)

        A = np.array([[1,dt],[0,1]]) # dynamics given change in time
        H = np.array([[1, dt]]) # measurement equation

        # prune too large of radii / any other outliers and just propagate dynamics! 

        if (x_meas**2+y_meas**2+z_meas**2)**0.5 > 1.75: 
            # just propagate dynamics
            x_p = (A.dot(self.x_m)).reshape((2,1))+np.array([(gm[0]/2)*dt*dt,gm[0]*dt]).reshape((2,1)) # affine dynamics added
            y_p = (A.dot(self.y_m)).reshape((2,1))+np.array([(gm[1]/2)*dt*dt,gm[1]*dt]).reshape((2,1)) # affine dynamics added
            z_p = (A.dot(self.z_m).reshape((2,1))+np.array([(gm[2]/2)*dt*dt,gm[2]*dt]).reshape((2,1))) # affine dynamics added

            # predict variance
            Px_p = A.dot(self.Px_m).dot(A.T)+Q
            Py_p = A.dot(self.Py_m).dot(A.T)+Q
            Pz_p = A.dot(self.Pz_m).dot(A.T)+Q

            self.Px_m = Px_p
            self.x_m = x_p

            self.Py_m = Py_p
            self.y_m = y_p

            self.Pz_m = Pz_p
            self.z_m = z_p

        else: 
            # standard kalman filter! 

            # predict step with affine dynamics added to z

            x_p = (A.dot(self.x_m)).reshape((2,1))+np.array([(gm[0]/2)*dt*dt,gm[0]*dt]).reshape((2,1)) # affine dynamics added
            y_p = (A.dot(self.y_m)).reshape((2,1))+np.array([(gm[1]/2)*dt*dt,gm[1]*dt]).reshape((2,1)) # affine dynamics added
            z_p = (A.dot(self.z_m).reshape((2,1))+np.array([(gm[2]/2)*dt*dt,gm[2]*dt]).reshape((2,1))) # affine dynamics added

            # predict variance
            Px_p = A.dot(self.Px_m).dot(A.T)+Q
            Py_p = A.dot(self.Py_m).dot(A.T)+Q
            Pz_p = A.dot(self.Pz_m).dot(A.T)+Q

            # Measurement update
            self.Px_m = np.linalg.inv(np.linalg.inv(Px_p) + H.T.dot(np.linalg.inv(Rx)).dot(H))
            self.x_m = x_p + self.Px_m.dot(H.T).dot(np.linalg.inv(Rx)).dot(x_meas-H.dot(x_p))

            self.Py_m = np.linalg.inv(np.linalg.inv(Py_p) + H.T.dot(np.linalg.inv(Ry)).dot(H))
            self.y_m = y_p + self.Py_m.dot(H.T).dot(np.linalg.inv(Ry)).dot(y_meas-H.dot(y_p))

            self.Pz_m = np.linalg.inv(np.linalg.inv(Pz_p) + H.T.dot(np.linalg.inv(Rz)).dot(H))
            self.z_m = z_p + self.Pz_m.dot(H.T).dot(np.linalg.inv(Rz)).dot(z_meas-H.dot(z_p))


        ## END STATE ESTIMATION, BEGIN PLOTTING / PUBLISHING 
        
        # check if the robot's sphere is going to be hit, and if pretty sure of this start publishing this location
        t_hit = self.intersection_time_solver(listener)
        #print t_hit
        # PRINT ESTIMATED LOCATION OF BALL
        location = self.intersection_solver(0.0001) 
        #vel = np.array([1,0,0])
        vel = location[:,1]
        vel_n = vel/np.linalg.norm(vel)
        #print vel_n
        rand_vec = np.array([20,0,0])
        proj = rand_vec-rand_vec.dot(vel_n)*vel_n
        if np.linalg.norm(proj) < .05: # check to make sure vectors aren't aligned 
            rand_vec = np.array([0,20,0])
            proj = rand_vec-rand_vec.dot(vel_n)*vel_n
        proj_n = proj/np.linalg.norm(proj)
        vel_y = np.cross(vel_n, proj_n)

        rot_matrix = np.zeros((4,4))
        rot_matrix[:3,:3] = np.vstack((vel_y,vel_n,proj_n)).T
        rot_matrix[3,3] = 1

        quat = tf.transformations.quaternion_from_matrix(rot_matrix)
        location[:,0] += -vel_n*self.wrist_distance

        msg_out_cam = PoseStamped(
            pose = Pose(
              position = Point(x = self.x_m[0], y=self.y_m[0],z=self.z_m[0]),
              #orientation = Quaternion(x = self.x_m[1], y=self.y_m[1],z=self.z_m[1], w=1)),
              orientation = Quaternion(x = -0.5, y=-0.5,z=-0.5, w=0.5)),
               
                  # header = Header(stamp=message.header.stamp, seq = self.hit_counter, frame_id = 0)
              header = Header(stamp=timestamp, seq=self.hit_counter, frame_id=fid) # 'base_link'


            # pose = Pose(
            #     position = Point(x = location[0,0], y=location[1,0],z=location[2,0]),
            #     orientation = Quaternion(x = quat[0], y=quat[1],z=quat[2], w=quat[3])),
            #     # header = Header(stamp=message.header.stamp, seq = self.hit_counter, frame_id = 0)
            #     header = Header(stamp=timestamp, seq=self.hit_counter, frame_id=fid) # 'kinect_link'
                )

        ###
        # Philipp Added for Trajectory optimization
        ###
                                
        t_start = 0.0001
        t_end = 0.5
        num_points = 100
        geom_points_list = []
        for ti in np.linspace(t_start, t_end, num=num_points):
            to_add_point = Point()
            pos_vel = self.intersection_solver(ti)
            to_add_point.x = pos_vel[0,0]
            to_add_point.y = pos_vel[1,0]
            to_add_point.z = pos_vel[2,0]
            geom_points_list.append(to_add_point)
        self.pub_trajectory(geom_points_list)
        print("Published Trajectory")

        try:
            self.pub.publish(listener.transformPose('base_link', msg_out_cam))
                    # print t_hit
            #print location
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException) as e:
            pass
            #print e

        # # ORIGINAL HIT PRINTING CODE
        # if t_hit: # update the hit counter. 2+ hits usually means it is pretty well locked on
        #   print t_hit
        #   self.hit_counter +=1
        #   if (self.hit_counter>self.hit_threshold):

        #       location = self.intersection_solver(t_hit) # 3x2
        #       #print t_hit
        #       #print self.z_m
        #       #print location
        #       # location = camera_to_zgravity_rot.dot(location)
        #       # location[0,0] -= self.base_trans[0]
        #       # location[1,0] -= self.base_trans[1]
        #       # location[2,0] -= self.base_trans[2]


        #       # Hacks to align the velocity vector along the z axis of the camera frame 
        #       vel = np.array([1,0,0])

        #       #vel = -location[:,1]
        #       vel_n = vel/np.linalg.norm(vel)
        #       rand_vec = np.array([20,0,0])
        #       proj = rand_vec-rand_vec.dot(vel_n)*vel_n
        #       if np.linalg.norm(proj) < .05: # check to make sure vectors aren't aligned 
        #           rand_vec = np.array([0,20,0])
        #           proj = rand_vec-rand_vec.dot(vel_n)*vel_n
        #       proj_n = proj/np.linalg.norm(proj)
        #       vel_y = np.cross(vel_n, proj_n)

        #       rot_matrix = np.zeros((4,4))
        #       #rot_matrix[:3,:3] = np.vstack((proj_n,vel_y,vel_n)).T
        #       rot_matrix[:3,:3] = np.vstack((vel_y,vel_n,proj_n)).T
        #       rot_matrix[3,3] = 1

        #       quat = tf.transformations.quaternion_from_matrix(rot_matrix)

        #       # update newest location 
        #       # location is defined in terms of the camera frame. velocity is defined in terms of the camera frame. 
        #       # vel_n is the velocity vector in terms of the camera frame 


        #       location[:,0] += -vel_n*self.wrist_distance





        #       # publish this location!!!!!!!!!!!!!!! (but transform to base_link frame first!)
        #       msg_out_cam = PoseStamped(
        #           pose = Pose(
        #               position = Point(x = location[0,0], y=location[1,0],z=location[2,0]),
        #               orientation = Quaternion(x = quat[0], y=quat[1],z=quat[2], w=quat[3])),
        #           # header = Header(stamp=message.header.stamp, seq = self.hit_counter, frame_id = 0)
        #           header = Header(stamp=timestamp, seq=self.hit_counter, frame_id=fid) # 'base_link'
        #           )


        #       ###
        #       # Philipp Added for Trajectory optimization
        #       ###
                                
  #               t_start = 0
        #       t_end = 0.5
        #       num_points = 100
        #       geom_points_list = []
        #       for ti in np.linspace(t_start, t_end, num=num_points):
        #           to_add_point = Point()
        #           pos_vel = self.intersection_solver(ti)
        #           to_add_point.x = pos_vel[0,0]
        #           to_add_point.y = pos_vel[1,0]
        #           to_add_point.z = pos_vel[2,0]
        #           geom_points_list.append(to_add_point)
        #       self.pub_trajectory(geom_points_list)
        #       print("Published Trajectory")
        #       ###
        #       try:
        #           self.pub.publish(listener.transformPose('base_link', msg_out_cam))
        #           # print t_hit
        #           print location
        #       except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ROSInterruptException) as e:
        #           print e

def listener():
    from time import sleep
    sleep(1)

    rospy.init_node('ball_filter', anonymous=True)

    tf_listener = tf.TransformListener()
    rate = rospy.Rate(30.0)
    sleep(3)
    frame_id = "kinect_link"
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0, 0, 0),   tf.transformations.quaternion_from_euler(0, 0, 0),
    #  rospy.Time(), frame_id, 'kinect_link')

    # base_trans, base_rot = tf_listener.lookupTransform('base_link',frame_id, rospy.Time())
    pub = rospy.Publisher('/final_ball_pose', PoseStamped, queue_size=1)
    # bkf = BallKalmanFilter(pub, base_trans, base_rot)
    bkf = BallKalmanFilter(pub,tf_listener)

    while not rospy.is_shutdown():
        try:
            bkf.marker_pub.publish(bkf.marker) # Sphere visualization code
            tf_time = tf_listener.getLatestCommonTime(frame_id, "ball")
            ball_trans, ball_rot = tf_listener.lookupTransform(frame_id, "ball", rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        bkf.callback(ball_trans, tf_time, frame_id, tf_listener)
        rate.sleep()  # Check if this is necessary

    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    # rospy.spin()


    #Python's syntax for a main() method
if __name__ == '__main__':
    listener()
