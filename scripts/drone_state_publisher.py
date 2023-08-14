#!/usr/bin/env python3

from threading import Thread,Event ,Lock 
from queue import Queue
import time
import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
import math
from cflib.crazyflie import syncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
import rclpy
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped, Vector3
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Path

#drone radios channels

N = 'radio://0/100/2M/E7E7E7E700' # N
V = 'radio://0/100/2M/E7E7E7E701' # v
F = 'radio://0/100/2M/E7E7E7E707'  # F 
Y = 'radio://0/100/2M/E7E7E7E706' # Y
P = 'radio://0/100/2M/E7E7E7E704'
W = 'radio://0/100/2M/E7E7E7E703'
# enter the uris need to be in swarm
uris = [V,Y,W]
# enter the mission and activate the required drones

drone = [[0,0,1],
         [1,1,0]]

#         [1,1,1]]


payload_idx = [0,1]

# add mission pickup and drop off loactions
mission_position = [[[1.1,0],[1,0]],[[0,0],[-1,0]]]
                    #,[[1,1],[-0.5,0.3]]]

# adjust the distance between 2 and 3 drones
distance_2 = 0.192
distance_3 = 0.5

velocity = 0.25


class MinimalPublisher(Node):
    
    global uris
    global drone
    global payload_idx
    global mission_position
    global distance_2
    global distance_3
    global velocity

    def __init__(self):
        super().__init__('minimal_publisher')
        cflib.crtp.init_drivers()
        self.factory = CachedCfFactory(rw_cache='./cache')
        self.publishernode()
        self.SubcriberNode()
        self.start()

    def publishernode(self):

        self.publisher_ = self.create_publisher(Int32, 'num_drones', 10)
        self.publisher_1 = self.create_publisher(Float64MultiArray,'drones_states',10)
        self.publisher_active =self.create_publisher(Int32MultiArray,'drones_active',10)
        self.publisher_waypoints=self.create_publisher(Float64MultiArray,'drones_goals',10)
        self.publisher_radii = self.create_publisher(Float64MultiArray,'drones_radii',10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.position_data = dict()
        self.waypoint_data =dict()
        self.path_dict =dict()
        self.path_found =dict()
        self.position_data_final = dict()

        self.float_list = Float64MultiArray()
        self.drone_active_list=Int32MultiArray()
        self.waypoint_publisher = Float64MultiArray()
        self.radius_list = Float64MultiArray()
        self.number_drones=len(uris)
        self.float_list.data = [0.0]*4*self.number_drones
        self.drone_active_list.data =[1]*self.number_drones
        self.radius_list.data =[0.15]*self.number_drones
        self.waypoint_publisher.data = [0.0]*2*self.number_drones
    
    def timer_callback(self):
        msg = Int32()
        self.number_drones=len(uris)
        msg.data = self.number_drones
        self.publisher_.publish(msg)

        self.publisher_active.publish(self.drone_active_list)

        self.publisher_radii.publish(self.radius_list)

        for i in range(len(payload_idx)):
            mission = self.mission_logger.get(payload_idx[i])

            if (mission[1][0]==4):

                if(len(mission[0])==1):
                    data = self.position_data.get(mission[0][0])
                    self.position_data_final.update({mission[0][0]:data})
                elif(len(mission[0])==4):
                    data = []
                    data = self.path_drones_2(mission[0][0],mission[0][1])
                    self.position_data_final.update({mission[0][0]:data})
                    self.position_data_final.update({mission[0][1]:data})
                  
                elif(len(mission[0])==5):
                    data = []
                    data = self.path_drones_3(mission[0][0],mission[0][1],mission[0][2])
                    self.position_data_final.update({mission[0][0]:data})
                    self.position_data_final.update({mission[0][1]:data})
                    self.position_data_final.update({mission[0][2]:data})

            else:
                for j in range(len(mission[0])):
                    if mission[0][j] == None:
                        continue
                    else:
                        data = self.position_data.get(mission[0][j])
                        self.position_data_final.update({mission[0][j]:data})
                

        # go through each missions drones and publish data into a new list and new dict
        # and publish from there than from position_data.get 

        for i in range(self.number_drones):
            try:
                self.float_list.data[i*4] = self.position_data_final.get(uris[i])[0]
                self.float_list.data[i*4+1] =self.position_data_final.get(uris[i])[1]
                self.float_list.data[i*4+2] = self.position_data_final.get(uris[i])[2]
                self.float_list.data[i*4+3] = self.position_data_final.get(uris[i])[3]
            except:
                print("skipping once")
        #print(self.float_list)
        self.publisher_1.publish(self.float_list)

        for i in range(self.number_drones):
            try:
                self.waypoint_publisher.data[i*2] =   self.waypoint_data.get(uris[i])[0]
                self.waypoint_publisher.data[i*2+1] = self.waypoint_data.get(uris[i])[1]
            except:
                print("skipping waypoint once")
        #print(self.waypoint_publisher)
        self.publisher_waypoints.publish(self.waypoint_publisher)
        self.Transform_Publisher()

    def Transform_Publisher(self):
        for i in range(self.number_drones):
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = 'odom'
            tf_msg.child_frame_id = str("drone")+str(i)
            tf_msg.header.stamp = self.get_clock().now().to_msg()

            try:
                tf_msg.transform.translation.x = self.position_data.get(uris[i])[0]
                tf_msg.transform.translation.y = self.position_data.get(uris[i])[1]
                #tf_msg.transform.translation.y = self.position_data.get(uris[i])[1]
        

                self.tf_broadcaster.sendTransform(tf_msg)
            except:
                print("skipping first trnsform publishin")


    """
    subcriber node 
    """

    def SubcriberNode(self):
        self.topics_create()
        print(self.path_topics)
        print(self.path_found_topic)
        self.current_path = []
        self.subcriber_list = []
        self.subcriber_path_found_list =[]
        self.path_found_list=[]
        for i in range(self.number_drones):

            self.current_path.append([1, 1])
            self.path_found_list.append(0)
            
        for i in range(0, self.number_drones):
            print("creating subscription")
            subscriber_path = self.create_subscription(
                Path,
                self.path_topics[i],
                self.path_callback_generator(i),
                10
            )
            self.subcriber_list.append(subscriber_path) 
            subscriber_path_found = self.create_subscription(
                Bool,
                self.path_found_topic[i],
                self.update_path_found_generator(i),
                10
            ) 
            self.subcriber_path_found_list.append(subscriber_path_found)

    def topics_create(self):
        self.path_topics=[]
        self.path_found_topic = []   
        for i in range(self.number_drones):
            topic=str('/drone_path')+str(i)
            found=str('/drone_path')+str(i)+str('found')
            self.path_topics.append(topic)
            print(found)
            print(topic)
            self.path_found_topic.append(found)


    def path_callback_generator(self, i):
        print("entered call back generator")
        def callback(msg):
            print(f'in call back {i}')
            self.current_path[i] = []
            for pose in msg.poses:
                #self.get_logger().info(f'Received path {(pose.pose.position.x, pose.pose.position.y)}')
                self.current_path[i].append([pose.pose.position.x, pose.pose.position.y])
        
            print(f"drone{i} : {self.current_path}")
            
        print("returning callback")
        return callback
    
    def update_path_found_generator(self,i):
        def callback1(msg):
            
            #self.get_logger().info(f'path found {(msg.data)}')
            self.path_found_list[i]= msg.data
            print(self.path_found_list)
        return callback1

    """
    drones code
    """

    def reset(self):
        self.swarm_.reset_estimators()

    def missions(self):
        self.drone_planner=dict()
        self.mission_logger = dict()
        for i in range(len(uris)):
            self.drone_planner.update({uris[i]:[[-1]]})
        for i in range(len(payload_idx)):
            data=[[None],[-1]]
            self.mission_logger.update({payload_idx[i]:data})
        for i in range(len(payload_idx)):
            for j in range(len(drone[i])):
                if(drone[i][j]==1):
                    x = self.drone_planner.get(uris[j])
                    if(x[0][0]==-1):
                        x = [[payload_idx[i]]]
                    else:
                        x.append([payload_idx[i]])
                    self.drone_planner.update({uris[j]:x})
            if(sum(drone[i])==2):
                self.setpoints_pickup_2(payload_idx[i])
            elif(sum(drone[i])==3):
                self.setpoints_pickup_3(payload_idx[i])
        print("missions done")
        print(self.drone_planner)
        print(self.mission_logger)
    
    def missions_checker(self):
        self.drones_complete = 0
        for i in range(len(uris)):
            total_mission = self.drone_planner.get(uris[i]) 

            if(len(total_mission)==0):
                self.drones_complete += 1 
    
            else:   
                first_mission = total_mission[0][0]
                if(len(mission_position[first_mission])==1):
                    mission = self.mission_logger.get(first_mission)
                    if(mission[1][0]==3):
                        if(len(mission[0])==2):
                            print("starting 2 drones")
                            mission[1][0]=4
                            data = mission[0]
                            drone_set_point = mission_position[first_mission][0]
                            data.append(drone_set_point[0])
                            data.append(drone_set_point[1])
                            print(data)
                            self.dropoff_2_waypoint(data)
                            #pickup 2 drones

                        elif(len(mission[0])==3):
                            print("starting 3 drones")
                            data = mission[0]
                            #pickup 3 drones
                            drone_set_point = mission_position[first_mission][0]
                            data.append(drone_set_point[0])
                            data.append(drone_set_point[1])
                            mission[1][0]=4
                            self.dropoff_3_waypoint(data)

                        elif(len(mission[0])==1):
                            drone_set_point = mission_position[first_mission][0]
                            data = [uris[i],drone_set_point[0],drone_set_point[1]]
                            self.setpoints_pickup_1(data)
                            mission[1][0]=4
                        self.mission_logger.update({first_mission:mission})    

                elif(len(mission_position[first_mission])==2):
                    mission_data = self.mission_logger.get(first_mission)
                    drones_used = mission_data[0]
                    mission_logger_status = mission_data[1]
                    if drones_used[0] == None:
                        drones_used = uris[i]
                        mission_logger_status = [0]
                        self.mission_logger.update({first_mission:[[drones_used],mission_logger_status]})
                        drone_set_point=mission_position[first_mission][0]
                        data = [uris[i],drone_set_point[0],drone_set_point[1]]
                        self.setpoints_pickup_1(data)
                        mission_position[first_mission].pop(0)

                    elif uris[i] in drones_used:
                        continue
                    else:
                        drones_used.append(uris[i])
                        mission_logger_status = [0]
                        self.mission_logger.update({first_mission:[drones_used,mission_logger_status]})
                        drone_set_point=mission_position[first_mission][0]
                        data = [uris[i],drone_set_point[0],drone_set_point[1]]
                        self.setpoints_pickup_1(data) 
                        mission_position[first_mission].pop(0)

                else:
                    mission_data = self.mission_logger.get(first_mission)
                    drones_used = mission_data[0]
                    mission_logger_status = mission_data[1]
                    if drones_used[0] == None:
                        drones_used = uris[i]
                        self.mission_logger.update({first_mission:[[drones_used],mission_logger_status]})
                        drone_set_point=mission_position[first_mission][0]
                        data = [uris[i],drone_set_point[0],drone_set_point[1]]
                        self.setpoints_pickup_1(data)
                        #print(mission_position[first_mission])
                        mission_position[first_mission].pop(0)

                    elif uris[i] in drones_used:
                        continue
                    else:
                        drones_used.append(uris[i])
                        self.mission_logger.update({first_mission:[drones_used,mission_logger_status]})
                        drone_set_point=mission_position[first_mission][0]
                        data = [uris[i],drone_set_point[0],drone_set_point[1]]
                        self.setpoints_pickup_1(data) 
                        mission_position[first_mission].pop(0)


    def start(self):
        print("started")
        self.missions() # missions assigned to each drone
        self.missions_checker()
        with Swarm(uris, factory=self.factory) as swarm:
            print("in_reset")
            swarm.reset_estimators()
            print("reset done")
            self.swarm_ =swarm 
            swarm.parallel_safe(self.simple_log_async)
            swarm.parallel_safe(self.take_off)
            self.pickup_complete_list = dict()
            self.seq_list_creator()
            print("done init")
            data = True
            while(data):
                print("in while loop")
                rclpy.spin_once(self)
                self.missions_checker()
                
                print(f'total drones completed mission: {self.drones_complete}')
                if(self.drones_complete==len(uris)):
                    data = False
                
                self.mission_distance_checker()
                seq_=self.seq()
                swarm.parallel_safe(self.run_sequence, args_dict=seq_)
            swarm.parallel_safe(self.land)
    



    def log_stab_callback(self,scf,timestamp, data, logconf):
   
        self.x = float(data['stateEstimate.x'])
        self.y = float(data['stateEstimate.y'])
        self.vx = float(data['stateEstimate.vx'])
        self.vy = float(data['stateEstimate.vy'])
        self.z = float(data['stateEstimate.z'])
        
        self.position_data[scf] = [self.x,self.y,self.vx,self.vy,self.z,scf]
    

        self.timer_callback()   
        

    def simple_log_async(self,scf):
        lg_stab = LogConfig(name='StateEstimate', period_in_ms=200)
        lg_stab.add_variable('stateEstimate.x', 'float')
        lg_stab.add_variable('stateEstimate.y', 'float')
        lg_stab.add_variable('stateEstimate.vx', 'float')
        lg_stab.add_variable('stateEstimate.vy', 'float') 
        lg_stab.add_variable('stateEstimate.z','float')    
        print("added variables")
        cf=scf.cf
        cf.log.add_config(lg_stab)
        lg_stab.data_received_cb.add_callback(lambda t, d, l: self.log_stab_callback(cf.link_uri, t, d, l))
        lg_stab.start()

    
    
    def activate_led_bit_mask(self,scf):
        scf.cf.param.set_value('led.bitmask', 255)
        print("light on")

    def deactivate_led_bit_mask(self,scf):
        scf.cf.param.set_value('led.bitmask', 0)
        print("light off")

    def light_check(self,scf):
        self.activate_led_bit_mask(scf)
        time.sleep(1)
        self.deactivate_led_bit_mask(scf)

    def take_off(self,scf):
        commander= scf.cf.high_level_commander
        commander.takeoff(1, 2.0)
        time.sleep(3)

    def land(self,scf):
        commander= scf.cf.high_level_commander
        commander.land(0.0, 2.0)
        time.sleep(2)
        commander.stop()
    

    def setpoints_splitter(self):
        distance_max=1
        distance_min=0.1
        results_list = []

        for i in range(self.number_drones):
            results_list.append([[self.position_data_final.get(uris[i])[0],self.position_data_final.get(uris[i])[1]]])
        print(f'result{results_list}')

        for i in range(self.number_drones):
            if self.path_found_list[i] == True:
                set_pts = self.current_path[i]
                print(set_pts)
                x = self.position_data.get(uris[i])[0]
                y = self.position_data.get(uris[i])[1]
                distance_setpoints_min= 100000
                index=0
                update=0
                for j in range(0,len(set_pts)):
                    if(len(set_pts)>2):
                        distance_between_setpoint = (x-set_pts[j][0])**2 + (y-set_pts[j][1])**2
                        print(distance_between_setpoint)
                        if(distance_between_setpoint>distance_min**2 and distance_between_setpoint<distance_setpoints_min):
                            distance_setpoints_min=distance_between_setpoint
                            results_list[i]=[[set_pts[j][0],set_pts[j][1]]]
                            index=j
                            update=1
                    if(len(set_pts)==2):
                        index = len(set_pts)-1
                if(index==len(set_pts)-1):
                    results_list[i]=[[self.waypoint_data.get(uris[i])[0],self.waypoint_data.get(uris[i])[1]]]
                    print("adding final way point")
                elif update==1:
                    print(index,len(set_pts))
                    results_list[i]=[[set_pts[index+1][0],set_pts[index+1][1]]]
                    print("adding final way point")
                
        print(f'updated list{results_list}')
        return results_list


    def seq_list_creator(self):
        self.seq_list = []
        for i in range(self.number_drones):
            self.seq_list.append('sequence'+str(i))

       
    def seq(self):   
        setpoints_list = self.setpoints_splitter()
        print("in seq")
        #print(setpoints_list)
        duration = 10

        for i in range(len(payload_idx)):
            mission = self.mission_logger.get(payload_idx[i])
            print(f'mission status:{mission[1][0]}')

            if(mission[1][0]==6):
                continue
            elif(mission[1][0]==-1):
                continue
            elif(mission[1][0]==0):
                print("going to pickup")
                up_distance = 1
                for j in range(len(mission[0])):   
                    index = uris.index(mission[0][j])
                    distance_check  = self.vertical_distance(uris[index],up_distance,0.02)
                    if distance_check == 1:
                        x = setpoints_list[index][0][0]
                        y = setpoints_list[index][0][1]
                        z = up_distance
                        duration = self.duration_calculator(uris[index],x,y,z) 
                        self.seq_list[index] = [(x,y,z,0,duration)]
                    else:
                        current_x = self.position_data.get(uris[index])[0]
                        current_y = self.position_data.get(uris[index])[1]
                        z = up_distance
                        duration = self.duration_calculator(uris[index],current_x,current_y,z) 
                        self.seq_list[index] = [
                            (current_x,current_y,up_distance,0,duration)
                        ]

            elif(mission[1][0]==1):
                print("picking up")
                
                check = 1
                for j in (range(len(mission[0]))):
                    if len(mission[0])==1:
                        down_distance = 0.08
                        index = uris.index(mission[0][j])
                        x = self.waypoint_data.get(uris[index])[0]
                        y = self.waypoint_data.get(uris[index])[1]
                        z = down_distance
                        duration = 1.5 * self.duration_calculator(uris[index],x,y,z)
                        self.seq_list[index] = [
                            (self.waypoint_data.get(uris[index])[0],self.waypoint_data.get(uris[index])[1],down_distance,0,duration)
                    ]
                    
                    else:
                        down_distance = 0.50
                        index = uris.index(mission[0][j])
                        x = self.waypoint_data.get(uris[index])[0]
                        y = self.waypoint_data.get(uris[index])[1]
                        z = down_distance
                        duration = 1.5 * self.duration_calculator(uris[index],x,y,z)
                        self.seq_list[index] = [
                            (self.waypoint_data.get(uris[index])[0],self.waypoint_data.get(uris[index])[1],down_distance,0,duration)]
                        
                    if j==0:
                        index_0 = index
                        self.drone_active_list.data[index] = 1
                    else:
                        self.drone_active_list.data[index] = 0   
                    
                    vertical_check = self.vertical_distance(uris[index],down_distance,0.02)
                    check = check * vertical_check 

                if len(mission[0]) == 2:
                    self.radius_list.data[index_0] = 0.275
                elif len(mission[0]) == 3:
                    self.radius_list.data[index_0] = 0.4
                
                if check == 1:
                    mission[1][0]=2
                self.mission_logger.update({i:mission})

            elif(mission[1][0]==2):
                print("pickup done")
                check = 1
                up_distance = 1
                for j in range(len(mission[0])):
                    index = uris.index(mission[0][j])
                    x = self.waypoint_data.get(uris[index])[0]
                    y = self.waypoint_data.get(uris[index])[1]
                    z = up_distance
                    duration =  2.2 * self.duration_calculator(uris[index],x,y,z)

                    self.seq_list[index] = [
                        (self.waypoint_data.get(uris[index])[0],self.waypoint_data.get(uris[index])[1],up_distance,0,duration)
                    ]
                    vertical_check = self.vertical_distance(uris[index],up_distance,0.1)
                    check = check * vertical_check
                if check ==1:
                    mission[1][0]=3
                self.mission_logger.update({i:mission})
                    # update way point
            
            elif(mission[1][0]==4):
                print("going to waypoint")
                up_distance = 1
                if(len(mission[0])==1):
                    index = uris.index(mission[0][0])
                    x = setpoints_list[index][0][0]
                    y = setpoints_list[index][0][1]
                    z = up_distance
                    duration =  self.duration_calculator(uris[index],x,y,z)
                    self.seq_list[index] = [
                        (setpoints_list[index][0][0],setpoints_list[index][0][1],up_distance,0,duration)
                    ]
                
                elif(len(mission[0])==4):
                    index  = uris.index(mission[0][0])
                    data = [setpoints_list[index][0][0],setpoints_list[index][0][1]]
                    final_xy = self.path_follower_2(data)
                    
                    x = data[0]
                    y = data[1]
                    z = up_distance
                    duration = 2.5 * self.duration_calculator(uris[index],x,y,z)
                    self.seq_list[index] = [
                        (final_xy[0], final_xy[1],up_distance,0,duration)
                    ]
                    index = uris.index(mission[0][1])
                    self.seq_list[index] = [
                        (final_xy[2],final_xy[3],up_distance,0,duration)
                    ]
                
                elif(len(mission[0])==5):


                    index  = uris.index(mission[0][0])

                    data = [setpoints_list[index][0][0],setpoints_list[index][0][1]]

                    x = data[0]
                    y = data[1]
                    z = up_distance
                    duration =  self.duration_calculator(uris[index],x,y,z)

                    final_xy = self.path_follower_3(data)
                    self.seq_list[index] = [
                        (final_xy[0], final_xy[1],z,0,duration)
                    ]
                    index = uris.index(mission[0][1])
                    self.seq_list[index] = [
                        (final_xy[2],final_xy[3],z,0,duration)
                    ]
                    index = uris.index(mission[0][2])
                    self.seq_list[index] = [
                        (final_xy[4],final_xy[5],z,0,duration)
                    ]
            elif(mission[1][0]==5):

                print("drop done")
                distance_down = 0.50
                check = 1
                self.mission_logger.update({i:mission})
                if(len(mission[0])==1):
                    index = uris.index(mission[0][0])
                    x = self.waypoint_data.get(uris[index])[0]
                    y = self.waypoint_data.get(uris[index])[1]
                    z = distance_down
                    duration =  self.duration_calculator(uris[index],x,y,z) 
                    self.seq_list[index] = [
                        (self.waypoint_data.get(uris[index])[0],self.waypoint_data.get(uris[index])[1],distance_down,0,duration)
                    ]
                    vertical_check = self.vertical_distance(uris[index],distance_down,0.02)
                    check = check * vertical_check

                elif(len(mission[0])==4):
                    distance = distance_2
                    index = uris.index(mission[0][0])
                    x = self.waypoint_data.get(uris[index])[0]
                    y = self.waypoint_data.get(uris[index])[1]
                    z= distance_down
                    duration = 2.5 * self.duration_calculator(uris[index],x,y,z) 
                    self.seq_list[index] = [
                        (x,y-distance,distance_down,0,duration)
                    ]
                    vertical_check = self.vertical_distance(uris[index],distance_down,0.05)
                    check = check * vertical_check
                    self.radius_list.data[index] = 0.15
                    index = uris.index(mission[0][1])
                    self.seq_list[index] = [
                        (x,y+distance,distance_down,0,duration)
                    ]
                    self.drone_active_list.data[index] = 1
                    self.radius_list.data[index] = 0.15
                  
                elif(len(mission[0])==3):
                    index = uris.index(mission[0][0])
                    x = self.waypoint_data.get(uris[index])[0]
                    y = self.waypoint_data.get(uris[index])[1]
                    self.seq_list[index] = [
                        (x,y,0.4,0,duration)
                    ]
                    self.radius_list.data[index] = 0.15

                    vertical_check = self.vertical_distance(uris[index],distance_down,0.05)
                    check = check * vertical_check

                    index = uris.index(mission[0][1])
                    self.seq_list[index] = [
                        (self.waypoint_data.get(uris[index])[0],self.waypoint_data.get(uris[index])[1],distance_down,0,duration)
                    ]
                    self.drone_active_list.data[index] = 1
                    self.radius_list.data[index] = 0.15

                    vertical_check = self.vertical_distance(uris[index],distance_down,0.05)
                    check = check * vertical_check

                    index = uris.index(mission[0][2])
                    self.seq_list[index] = [
                        (self.waypoint_data.get(uris[index])[0],self.waypoint_data.get(uris[index])[1],distance_down,0,duration)
                    ]
                    self.drone_active_list.data[index] = 1
                    self.radius_list.data[index] = 0.15

                    vertical_check = self.vertical_distance(uris[index],distance_down,0.05)
                    check = check * vertical_check
                if check == 1:
                    mission[1][0] = 6

                    poped_element = payload_idx[i]
                    # mission_position.pop(i)
                    for  j in range(len(drone[poped_element])):
                        if drone[poped_element][j]==1:
                            drone_mission = self.drone_planner.get(uris[j])
                            drone_mission.pop(0)
                            self.drone_planner.update({uris[j]:drone_mission})
                    


        seq_args = dict()
        for i in range(self.number_drones):
            seq_args.update({uris[i]:[self.seq_list[i]]})

        return seq_args


    def run_sequence(self,scf: syncCrazyflie.SyncCrazyflie, sequence):
        cf = scf.cf


        for arguments in sequence:
            commander = scf.cf.high_level_commander


            x, y, z = arguments[0], arguments[1], arguments[2]
            yaw = arguments[3]
            duration = arguments[4]

            print('Setting position {} to cf {}'.format((x, y, z,yaw), cf.link_uri))
            #commander.set_default_velocity(0.01)
            commander.go_to(x, y, z, yaw, duration, relative=False)
            time.sleep(1)

    def path_drones_3(self,uri_1,uri_2,uri_3): # 
        y1 = self.position_data.get(uri_1)[1]
        x1 = self.position_data.get(uri_1)[0]
        y2 = self.position_data.get(uri_2)[1]
        x2 = self.position_data.get(uri_2)[0]
        y3 = self.position_data.get(uri_3)[1]
        x3 = self.position_data.get(uri_3)[0]


        vx1 = self.position_data.get(uri_1)[2]
        vx2 = self.position_data.get(uri_2)[2]
        vx3 = self.position_data.get(uri_3)[2]

        vy1 = self.position_data.get(uri_1)[3]
        vy2 = self.position_data.get(uri_2)[3]
        vy3 = self.position_data.get(uri_3)[3]

        cx = (x1+x2+x3)/3
        cy = (y1+y2+y3)/3
        
        cvx = (vx1 + vx2 + vx3)/3
        cvy = (vy1 + vy2 + vy3)/3

        return [cx,cy,cvx,cvy]
    
    def path_drones_2(self,uri_1,uri_2):
        y1 = self.position_data.get(uri_1)[1]
        y2 = self.position_data.get(uri_2)[1]

        x1 = self.position_data.get(uri_1)[0]
        x2 = self.position_data.get(uri_2)[0]

        vx1 = self.position_data.get(uri_1)[2]
        vx2 = self.position_data.get(uri_2)[2]

        vy1 = self.position_data.get(uri_1)[3]
        vy2 = self.position_data.get(uri_2)[3]
        
        cx = (x1+x2)/2
        cy = (y1+y2)/2
        cvx = (vx1+vx2)/2
        cvy = (vy1+vy2)/2

        return [cx,cy,cvx,cvy]

    def dropoff_2_waypoint(self,data):
        distance = distance_2
        cx = data[2]
        cy = data[3]

        self.waypoint_data[data[0]] = [cx,cy]
        self.waypoint_data[data[1]] = [cx,cy]

    def dropoff_3_waypoint(self,data):
        #distance = 0.30
        cx=data[3]
        cy=data[4]
        """
        y1= cy-(distance)*math.cos(math.pi/6) #drone1 y
        x1= cx-distance/2  # drone1 x
        y2= cy+(distance)*math.cos(math.pi/6) #drone2 y
        x2= cx-distance/2 #drone2 x

        y3= cy #drone 3 y
        x3 = cx+distance
        y3= cy #drone 3 y
        x3 = cx # drone 3 x
        """
        self.waypoint_data[data[0]]= [cx,cy]
        self.waypoint_data[data[1]]= [cx,cy]
        self.waypoint_data[data[2]]= [cx,cy]

    def setpoints_pickup_3(self,payload_nu):
        distance=distance_3 # in meters centroid to drone
        #cx=set_pts[i][0] # x setpoint
        #cy=set_pts[i][1] # y setpoint
        data = mission_position[payload_nu][0]
        final = mission_position[payload_nu][1]
        cx = data[0]
        cy = data[1]
        y1 = cy-(distance)*math.cos(math.pi/6) #drone1 y
        x1 = cx-distance/2  # drone1 x
        y2 = cy+(distance)*math.cos(math.pi/6) #drone2 y
        x2 = cx-distance/2 #drone2 x
        y3= cy #drone 3 y
        x3 = cx+distance # drone 3 x
        #yaw=0 # angle of drone
        data1 = [[x1,y1],[x2,y2],[x3,y3],final]
        mission_position[payload_nu] = data1
        #self.waypoint_data[uri_1]= [x1,y1]
        #self.waypoint_data[uri_2]= [x2,y2]
        #self.waypoint_data[uri_3]= [x3,y3]

    def setpoints_pickup_2(self,payload_nu):
        d=distance_2 # distance in meters
        data = mission_position[payload_nu][0]
        final = mission_position[payload_nu][1]
        cx=data[0]
        cy=data[1]
        x1=cx  # drone1 x
        x2=cx  # drone2 x
        y1=cy-d # drone1 y
        y2=cy+d # drone2 y
        yaw=0 # angle of drone
        data1 = [[x1,y1],[x2,y2],final]
        mission_position[payload_nu]=data1
    
    def setpoints_pickup_1(self,data):
        x=data[1]
        y=data[2]
        self.waypoint_data[data[0]]= [x,y]

    def path_follower_2(self,data):
        cx = data[0]
        cy = data[1] 
        distance = distance_2
        x1=cx  # drone1 x
        x2=cx  # drone2 x
        y1=cy-distance # drone1 y
        y2=cy+distance # drone2 y
        final_xy = [x1,y1,x2,y2]
        return final_xy
    
    def path_follower_3(self,data):
        distance = distance_3
        cx = data[0]
        cy = data[1]
        y1 = cy-(distance)*math.cos(math.pi/6) #drone1 y
        x1 = cx-distance/2  # drone1 x
        y2 = cy+(distance)*math.cos(math.pi/6) #drone2 y
        x2 = cx-distance/2 #drone2 x

        y3 = cy #drone 3 y
        x3 = cx+distance
        final_xy = [x1,y1,x2,y2,x3,y3]
        return final_xy
    
    def reached_final_setpoint(self,drone_uri):
        
        allowed_distance = 0.15
        
        current_position_drone_x = self.position_data_final.get(drone_uri)[0]
        current_position_drone_y = self.position_data_final.get(drone_uri)[1]

        distance_to_setpoint = math.sqrt((current_position_drone_x-self.waypoint_data[drone_uri][0])**2 +(current_position_drone_y-self.waypoint_data[drone_uri][1])**2)
        print(f'distnace is {drone_uri} : {distance_to_setpoint} : {current_position_drone_x,current_position_drone_y}')
        if distance_to_setpoint <= allowed_distance:
            drone_reached = 1
        else:
            drone_reached = 0
        return drone_reached

    def mission_distance_checker(self):
        for i in range(len(payload_idx)):
            mission = self.mission_logger.get(payload_idx[i])
            check = 1
            if mission[1][0]==0:
                print(mission)
                for j in range(len(mission[0])):
                    if mission[0][j] == None:
                        continue
                    else:  
                        temp = self.reached_final_setpoint(mission[0][j])
                        check = check*temp
                if check == 1:
                    mission[1][0]=1


            self.mission_logger.update({payload_idx[i]:mission})
            print(mission[0])
            if mission[1][0]==4:
                if(len(mission[0])==1):
                    if(self.reached_final_setpoint(mission[0][0])==1 ):
                        mission[1][0] = 5   

                elif(len(mission[0])==4):
                    if(self.reached_final_setpoint(mission[0][0])==1 or self.reached_final_setpoint(mission[0][1])==1):
                        
                        mission[1][0] = 5
                
                elif(len(mission[0])==5):
                    temp =(self.reached_final_setpoint(mission[0][1]))*(self.reached_final_setpoint(mission[0][2]))
                    if temp==1:
                        mission[1][0] = 5
                

                
            self.mission_logger.update({payload_idx[i]:mission})

    def vertical_distance(self,drone_uri,mention_distance,allowed_distance):

        allowed_distance = allowed_distance
        try:
            height = self.position_data.get(drone_uri)[4]
            distance_vertical = abs(height - mention_distance)

            if distance_vertical <=allowed_distance:
                drone_reached = 1
            else:
                drone_reached = 0 
        except:
            print("skipping height once")
            drone_reached = 0
        
        return drone_reached

    def duration_calculator(self,uri,x,y,z):
        current_x = self.position_data_final.get(uri)[0]
        current_y = self.position_data_final.get(uri)[1]
        cuurent_z = self.position_data.get(uri)[4]

        distance = math.sqrt((current_x-x)**2 + (current_y-y)**2 + (cuurent_z - z)**2)

        duration = distance/velocity

        return duration

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    print ("main")
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
