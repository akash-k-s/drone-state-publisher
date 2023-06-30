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
from nav_msgs.msg import Path
from std_msgs.msg import Bool

#drone radios channels

A = 'radio://0/100/2M/E7E7E7E700'
B = 'radio://0/100/2M/E7E7E7E701'
C = 'radio://0/100/2M/E7E7E7E707'  # F 
#D = 'radio://0/100/2M/E7E7E7E706' # Y

uris = [A,B,C]
lock=Lock()


class MinimalPublisher(Node):
    
    global uris
    #global Drone
    global event
    event = Event()
    global lock
    


    def __init__(self):
        super().__init__('minimal_publisher')

        self.publisher_ = self.create_publisher(Int32, 'num_drones', 10)
        self.publisher_1 = self.create_publisher(Float64MultiArray,'drones_states',10)
        self.publisher_active =self.create_publisher(Int32MultiArray,'drones_active',10)
        self.publisher_waypoints=self.create_publisher(Float64MultiArray,'drones_goals',10)
        self.publisher_radii = self.create_publisher(Float64MultiArray,'drones_radii',10)
        timer_period = 0.05 # seconds
        print("start")
        self.position_data = dict()
        self.waypoint_data =dict()
        self.float_list = Float64MultiArray()
        self.drone_active_list=Int32MultiArray()
        self.waypoint_publisher = Float64MultiArray()
        self.radius_list = Float64MultiArray()
        self.number_drones=len(uris)
        self.float_list.data = [0.0]*4*self.number_drones
        self.drone_active_list.data =[1]*self.number_drones
        self.radius_list.data =[0.15]*self.number_drones
        self.waypoint_publisher.data = [0.0]*2*self.number_drones
        cflib.crtp.init_drivers()
        self.factory = CachedCfFactory(rw_cache='./cache')
        self.SubscriberNode()
        self.start()

    def SubscriberNode(self):
        self.subscribers = []
        self.topics_create()
        for self.topic in self.path_topics:
            print(self.topic)
            subscriber_path = self.create_subscription(Path, self.topic, self.callback1, 10)
            #subscriber_path.topic_name = self.topic
            #self.subscribers.append(subscriber_path)
        self.get_logger().info(f'Subscribed to topics: {self.path_topics}')

        for self.topic in self.path_found_topic:
            print(self.topic)
            subscriber_path_found = self.create_subscription(Bool, self.topic, self.callback2, 10)
            #subscriber_path_found.topic_name = self.topic
            #self.subscribers.append(subscriber_path_found)

    
    def callback1(self, msg):
        topic = self.get_topic_name(msg)
        self.get_logger().info(f'Received message on topic {topic}: {msg.data}')

    def callback2(self, msg):
        topic = self.get_topic_name(msg)
        self.get_logger().info(f'Received message on topic {topic}: {msg.data}')

    def get_topic_name(self, msg):
        for subscriber in self.subscribers:
            if subscriber.topic_name in msg._full_text:
                return subscriber.topic_name
        return 'Unknown Topic'

    def topics_create(self):
        self.path_topics=[]
        self.path_found_topic = []   
        for i in range(self.number_drones):
            topic=str('/drone_path')+str(i)
            found=str('/drone_path')+str(i)+str('found')
            self.path_topics.append(topic)
            self.path_found_topic.append(found)

    def reset(self):
        self.swarm_.reset_estimators()


    def start(self):
        with Swarm(uris, factory=self.factory) as swarm:
            swarm.reset_estimators()
            print("reset done")
            self.swarm_ =swarm 
            swarm.parallel_safe(self.simple_log_async)
            #swarm.parallel_safe(self.take_off)
            #self.setpoints_pickup_3(uris[0],uris[1],uris[2])
            self.setpoints_pickup_2(uris[1],uris[2])
            self.setpoints_pickup_1(uris[0])
            while(1):
                self.setpoints_pickup_1(uris[0])
            #    swarm.parallel_safe(self.land)
    


    def log_stab_callback(self,scf,timestamp, data, logconf):
   
        self.x = float(data['stateEstimate.x'])
        self.y = float(data['stateEstimate.y'])
        self.vx = float(data['stateEstimate.vx'])
        self.vy = float(data['stateEstimate.vy'])
        
        self.position_data[scf] = [self.x,self.y,self.vx,self.vy,scf]
        print("got data")
        #time.sleep(0.7)
        self.timer_callback()   
        

    def simple_log_async(self,scf):
        lg_stab = LogConfig(name='StateEstimate', period_in_ms=200)
        lg_stab.add_variable('stateEstimate.x', 'float')
        lg_stab.add_variable('stateEstimate.y', 'float')
        lg_stab.add_variable('stateEstimate.vx', 'float')
        lg_stab.add_variable('stateEstimate.vy', 'float')     
        print("added variables")
        cf=scf.cf
        cf.log.add_config(lg_stab)
        lg_stab.data_received_cb.add_callback(lambda t, d, l: self.log_stab_callback(cf.link_uri, t, d, l))
        lg_stab.start()
        #time.sleep(2)
        #lg_stab.stop()

    def timer_callback(self):
        print("in timer")
        msg = Int32()
        self.number_drones=len(uris)
        msg.data = self.number_drones
        self.publisher_.publish(msg)
        #print("creating drones instance")

        self.publisher_active.publish(self.drone_active_list)

        self.publisher_radii.publish(self.radius_list)
        
        for i in range(self.number_drones):
            self.float_list.data[i*4] = self.position_data.get(uris[i])[0]
            self.float_list.data[i*4+1] =self.position_data.get(uris[i])[1]
            self.float_list.data[i*4+2] = self.position_data.get(uris[i])[2]
            self.float_list.data[i*4+3] = self.position_data.get(uris[i])[3]
        print(self.float_list)
        self.publisher_1.publish(self.float_list)

        for i in range(self.number_drones):
            self.waypoint_publisher.data[i*2] =   self.waypoint_data.get(uris[i])[0]
            self.waypoint_publisher.data[i*2+1] = self.waypoint_data.get(uris[i])[1]

        print(self.waypoint_publisher)
        self.publisher_waypoints.publish(self.waypoint_publisher)



    # seq_args = {

    #     uris[0]: [sequence0],
    #     uris[1]: [sequence1],
    #     uris[2]: [sequence2]
    # }
    """
    def run_sequence(scf: syncCrazyflie.SyncCrazyflie, sequence):
        cf = scf.cf

        for arguments in sequence:
            commander = scf.cf.high_level_commander

            x, y, z = arguments[0], arguments[1], arguments[2]
            yaw = arguments[3]
            duration = arguments[4]

            print('Setting position {} to cf {}'.format((x, y, z,yaw), cf.link_uri))
            #commander.set_default_velocity(0.2)
            commander.go_to(x, y, z, yaw, duration, relative=False)

            time.sleep(duration)
    """

    
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
        commander.takeoff(1.0, 2.0)
        time.sleep(3)

    def land(self,scf):
        commander= scf.cf.high_level_commander
        commander.land(0.0, 2.0)
        time.sleep(2)
        commander.stop()


    def setpoints_pickup_3(self,uri_1,uri_2,uri_3):
        distance=0.30 # in meters centroid to drone
        #cx=set_pts[i][0] # x setpoint
        #cy=set_pts[i][1] # y setpoint
        cx=0
        cy=0
        y1= cy-(distance)*math.cos(math.pi/6) #drone1 y
        x1= cx-distance/2  # drone1 x
        y2= cy+(distance)*math.cos(math.pi/6) #drone2 y
        x2= cx-distance/2 #drone2 x

        y3= cy #drone 3 y
        x3 = cx+distance # drone 3 x
        #yaw=0 # angle of drone
        self.waypoint_data[uri_1]= [x1,y1]
        self.waypoint_data[uri_2]= [x2,y2]
        self.waypoint_data[uri_3]= [x3,y3]

    def setpoints_pickup_2(self,uri_1,uri_2):
        d=0.5 # distance in meters
        cx=0 #pickup coordinates in x
        cy=0 #pickup coordinates in y 
        x1=cx  # drone1 x
        x2=cx  # drone2 x
        y1=cy-d # drone1 y
        y2=cy+d # drone2 y
        yaw=0 # angle of drone
        self.waypoint_data[uri_1]= [x1,y1]
        self.waypoint_data[uri_2]= [x2,y2] 
    def setpoints_pickup_1(self,uri_1):
        cx=1
        cy=1
        self.waypoint_data[uri_1]= [cx,cy]





def main(args=None):


    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()




    print ("main")
    rclpy.spin(minimal_publisher)
    print("spinninng")
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
