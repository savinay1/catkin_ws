#!/usr/bin/env python


import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,Waypoint,WaypointList,CommandCode 
from mavros_msgs.srv import WaypointPull,WaypointPush,WaypointClear,WaypointSetCurrent,CommandBool, SetMode
from sensor_msgs.msg import NavSatFix

import csv
import pdb
import math


num_uav = 3
min_thresh = 1
current_waypoints = [{'x_lat':0,'y_long':0,'z_alt':0} for i in range(num_uav)]
halt = [0,0,0]
home_location = [47.3977436, 8.5455934,535]
R = 6371

# callback method for state sub
current_state = [State() for i in range(num_uav)] 
offb_set_mode = SetMode
current_location = [NavSatFix() for i in range(num_uav)]
def state_cb1(state):
    global current_state
    current_state[0] = state

def state_cb2(state):
    global current_state
    current_state[1] = state

def state_cb3(state):
    global current_state
    current_state[2] = state

def current_wp1(wp_list):
    #pdb.set_trace()
    for wp in wp_list.waypoints:
        if wp.is_current == True :
            current_waypoints[0]['x_lat']= wp.x_lat
            current_waypoints[0]['y_long']= wp.y_long
            current_waypoints[0]['z_alt']= wp.z_alt
            return

def current_wp2(wp_list):
    for wp in wp_list.waypoints:
        if wp.is_current == True :
            current_waypoints[1]['x_lat']= wp.x_lat
            current_waypoints[1]['y_long']= wp.y_long
            current_waypoints[1]['z_alt']= wp.z_alt
            return


def current_wp3(wp_list):
    for wp in wp_list.waypoints:
        if wp.is_current == True :
            current_waypoints[2]['x_lat']= wp.x_lat
            current_waypoints[2]['y_long']= wp.y_long
            current_waypoints[2]['z_alt']= wp.z_alt
            return
            
def location_1(loc):
    global current_location
    #print loc.latitude,loc.longitude,loc.altitude
    current_location[0] = loc


def location_2(loc):
    global current_location
    #print loc.latitude,loc.longitude,loc.altitude
    current_location[1] = loc

def location_3(loc):
    global current_location
    #print loc.latitude,loc.longitude,loc.altitude
    current_location[2] = loc
    
local_pos_pub = rospy.Publisher("uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)
wp_uav1 = rospy.Subscriber("uav1/mavros/mission/waypoints",WaypointList,current_wp1)
wp_uav2 = rospy.Subscriber("uav2/mavros/mission/waypoints",WaypointList,current_wp2)
wp_uav3 = rospy.Subscriber("uav3/mavros/mission/waypoints",WaypointList,current_wp3)

state_sub1 = rospy.Subscriber("uav1/mavros/state", State, state_cb1)
state_sub2 = rospy.Subscriber("uav2/mavros/state", State, state_cb2)
state_sub3 = rospy.Subscriber("uav3/mavros/state", State, state_cb3)
loc_client_1= rospy.Subscriber("uav1"+"/mavros/global_position/global",NavSatFix,location_1 )
loc_client_2= rospy.Subscriber("uav2"+"/mavros/global_position/global",NavSatFix,location_2 )
loc_client_3= rospy.Subscriber("uav3"+"/mavros/global_position/global",NavSatFix,location_3 )

for n in range(num_uav):
    exec("wp_client_clear"+str(n)+" = rospy.ServiceProxy(\"uav"+str(n+1)+"/mavros/mission/clear\", WaypointClear)")
    exec("wp_client"+str(n)+ " = rospy.ServiceProxy(\"uav"+str(n+1)+"/mavros/mission/push\", WaypointPush)")



pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

convertor = {'FRAME_GLOBAL_REL_ALT':Waypoint.FRAME_GLOBAL_REL_ALT,
             'NAV_TAKEOFF':CommandCode.NAV_TAKEOFF,
             'NAV_LOITER_TIME':CommandCode.NAV_LOITER_TIME,
             'NAV_WAYPOINT':CommandCode.NAV_WAYPOINT,
             'NAV_RETURN_TO_LAUNCH':CommandCode.NAV_RETURN_TO_LAUNCH,
             'FRAME_MISSION':Waypoint.FRAME_MISSION,
             'true':True,
             'false':False
}

#wp_list = WaypointList()
#wp_list = [WaypointList() for i in range(num_uav)]
wp_list = []
def push_waypoints(mission_file):
    global wp_list
    #temp=[]
    #pdb.set_trace()
    wp_list = []
    with open(mission_file,'rb') as csvfile:
        print mission_file
        for rows in csv.reader(csvfile):
            wp = Waypoint()
            wp.frame = convertor[rows[0]]
            wp.command = convertor[rows[1]]
            wp.is_current = convertor[rows[2]]
            wp.autocontinue = convertor[rows[3]]
            wp.param1 = float(rows[4])
            wp.param2 = float(rows[5])
            wp.param3 = float(rows[6])
            wp.param4 = float(rows[7])
            wp.x_lat = float(rows[8])
            wp.y_long = float(rows[9])
            wp.z_alt = float(rows[10])
            #wp_list.waypoints.append(wp)
            wp_list.append(wp)
        #pdb.set_trace()
        #wp_list[i]=temp
def setArm(uav):
    rospy.wait_for_service(uav+'/mavros/cmd/arming')
    try:
        arming_client = rospy.ServiceProxy(uav+"/mavros/cmd/arming", CommandBool)
        arming_client(True)
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e

        
def setMode(uav,mode):
    rospy.wait_for_service(uav+'/mavros/set_mode')
    try:
        set_mode_client = rospy.ServiceProxy(uav+"/mavros/set_mode", SetMode)
        set_mode_client(base_mode=0, custom_mode=mode)
        return True
    except rospy.ServiceException, e:
        print "Service arm call failed: %s"%e
        return False
        
def setDisarm(uav):
   rospy.wait_for_service(uav+'/mavros/cmd/arming')
   try:
       arming_client = rospy.ServiceProxy(uav+"/mavros/cmd/arming", CommandBool)
       arming_client(False)
   except rospy.ServiceException, e:
       print "Service arm call failed: %s"%e


def setTakeoffMode(uav):
   rospy.wait_for_service(uav+'/mavros/cmd/takeoff')
   try:
       takeoffService = rospy.ServiceProxy(uav+'/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
       takeoffService(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "Service takeoff call failed: %s"%e



def setLandMode(uav):
   rospy.wait_for_service(uav+'/mavros/cmd/land')
   try:
       landService = rospy.ServiceProxy(uav+'/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
       #http://wiki.ros.org/mavros/CustomModes for custom modes
       isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
   except rospy.ServiceException, e:
       print "service land call failed: %s. The vehicle cannot land "%e
           


def timer_callback(event):
    check_collision()
    print "----------------"


def callb(event,uav):
    setMode(uav,"AUTO.MISSION")
    print "eeeeeeeeeeeeee"

def intersect(a1,b1,a2,b2):
    t = (a1-a2)/((b2-a2)-(b1-a1))
    return a1 + t*(b1-a1)

    
def collision_alert(i,j):
    uavi = 'uav'+str(i+1)
    uavj = 'uav'+str(j+1)
    curi = [R*(current_location[i].latitude-home_location[0])*math.pi/180*math.cos(home_location[1]),R*(current_location[i].longitude-home_location[1])*math.pi/180,current_location[i].altitude-home_location[2]]
    curj = [R*(current_location[j].latitude-home_location[0])*math.pi/180*math.cos(home_location[1]),R*(current_location[j].longitude-home_location[1])*math.pi/180,current_location[j].altitude-home_location[2]]
    nexti= [R*(current_waypoints[i]['x_lat']-home_location[0])*math.pi/180*math.cos(home_location[1]),R*(current_waypoints[i]['y_long']-home_location[1])*math.pi/180,current_waypoints[i]['z_alt']-home_location[2]]
    nextj= [R*(current_waypoints[j]['x_lat']-home_location[0])*math.pi/180*math.cos(home_location[1]),R*(current_waypoints[j]['y_long']-home_location[0])*math.pi/180,current_waypoints[j]['z_alt']-home_location[2]]

    intr = [intersect(curi[0],nexti[0],curj[0],nextj[0]),intersect(curi[1],nexti[1],curj[1],nextj[1]),intersect(curi[2],nexti[2],curj[2],nextj[2])]

    di = math.sqrt((curi[0]-intr[0])**2+(curi[1]-intr[1])**2+(curi[2]-intr[2])**2)
    dj = math.sqrt((curj[0]-intr[0])**2+(curj[1]-intr[1])**2+(curj[2]-intr[2])**2)

    if di>dj:
        setMode('uav'+str(i+1),"AUTO.LOITER")
        halt[i] = 1
        print "uav" + str(i+1) + " stopped" 
    else:
        setMode('uav'+str(j+1),"AUTO.LOITER")
        halt[j] = 1
        print "uav" + str(j+1) + " stopped"
    
    #pdb.set_trace()
    #s,a=wp_pull()
    #setMode(uav,"AUTO.LOITER")
    #rospy.Timer(rospy.Duration(2),callb,(uav),True)
    #setMode(uav_name,"AUTO.MISSION")
    pass

    
def check_collision():
    for i in range(num_uav):
        c_flag = 0
        for j in range(i+1,num_uav):
            dist = math.sqrt((R*(current_location[i].latitude-home_location[0])*math.pi/180*math.cos(home_location[1])-R*(current_location[j].latitude-home_location[0])*math.pi/180*math.cos(home_location[1]))**2 + (R*(current_location[i].longitude-home_location[1])*math.pi/180-R*(current_location[j].longitude-home_location[1])*math.pi/180)**2 + (current_location[i].altitude-current_location[j].altitude)**2)
            if dist<min_thresh:
                c_flag = 1
                #pdb.set_trace()
                if halt[i] == 0 and halt[j] == 0:
                    print "COLLISION AVOIDED between "+str(i+1) +" and " + str(j+1)
                    collision_alert(i,j)
        if c_flag == 0:
            if halt[i] == 1:
                setMode('uav'+str(i+1),"AUTO.MISSION")
                halt[i] = 0
                print "uav" + str(i+1) + " resumed"
            
            #print "distance between " + str(i) + " and " + str(j) + " is " + str(dist) 

       
def setWavePoints(uav,n):
    rospy.wait_for_service(uav+'/mavros/mission/push')
    try:
        #pdb.set_trace()
        
        #pdb.set_trace()
        exec("wp_client_clear"+str(n)+"()")
        exec("is_sen = wp_client"+str(n)+"(start_index = 0,waypoints = wp_list)")
        if is_sen :
            rospy.loginfo("MISSION SENT ")
            if current_state[n].mode != "AUTO.MISSION" :
                if setMode(uav,'AUTO.MISSION'):
                    rospy.loginfo("AUTO MISSION enabled")
    except rospy.ServiceException, e:
        print "service land call failed: %s. The vehicle cannot land "%e
        
def position_control():
    rospy.init_node('offb_node', anonymous=True)
    #prev_state = current_state
    rate = rospy.Rate(20.0) # MUST be more then 2Hz
    #rospy.Timer(rospy.Duration(2),timer_callback)
    # wait for FCU connection
    while not current_state[0].connected:
        rate.sleep()

    
    #setArm('uav1')
    #setArm('uav2')

    
    
    #pdb.set_trace()

    #setArm('uav1')
    
    push_waypoints('mission1.csv')
    setWavePoints('uav1',0)
    
    push_waypoints('mission2.csv')
    setWavePoints('uav2',1)

    push_waypoints('mission3.csv')
    setWavePoints('uav3',2)
    
    setArm('uav1')
    setArm('uav2')
    setArm('uav3')
    
    while not rospy.is_shutdown():
        check_collision()
        rate.sleep()
    
    '''
    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "AUTO.MISSION" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="AUTO.MISSION")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               setArm('uav1')
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        # Update timestamp and publish pose 
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

        setWavePoints('uav1')

       ''' 

if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
