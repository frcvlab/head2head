#
# Head to head discrete event simulation
# compares mapping/path-planning with WAVN in warehouse application
#
# robot class, implements MPP and WAVN motion strategies
#
# D. Lyons June 2025
#
#
import random
import math
import simpy
import numpy as np

import gates as gt # this is for the robot modelof the gates

DEBUG = True

# The robot can be ready to acquire, going to pickup, or going to dropoff
STATE_READY     = 1   # Ready to get the next product
STATE_2INFLOW   = 2   # going to the inflow q to get a product
STATE_2OUTFLOW  = 3   # delivering the product to the outflow

# the two main strategies to be tested
METHOD_MPP = 0      # mapping and path-planning based approach
METHOD_WAVN = 1     # wide area visual navigation appraoch

# names for the two robots in the facility
ROBOTA = 1
ROBOTB = 2

# Size of the facility, 100x100m
FACILITY_WIDTH  = 100 # meters
FACILITY_LENGTH = 100 # meters

# default locations for the two robots 
LOCATIONA = (20,50)     # meters
LOCATIONB = (100-20,50) # meters

# TIME PARAMETERS for the two robots
ROBOT_SPEED = 0.5 # constant speed, meters a second
ROBOT_DELIVER_TIME = 30 # seconds, time to drop off a product
ROBOT_FAIL_RETRIES = 20 # if blocked, how many times to try again
ROBOT_PP_TIME = 0.5     # how long to plan a path, in seconds
ROBOT_VH_DISTANCE = 5   # extra distance penalty for visual homing path
ROBOT_IDLE_TIME = 1.0   # waiting for product to arrive in seconds
ROBOT_LANDMARK_TIME = 1.0    # how long to do landmark detection, in seconds
ROBOT_EXPLORE_WAIT  = 300.0  # wait this long in the hope a gate will become open 

#measurements in meters for sensor ranges
LIDAR_RANGE = 30 # meters, this is common range for Lidar
CAMERA_RANGE = math.sqrt( 100*100+50*50 ) # see across its own half of facility

# Discrete event time chunks can be large, limit the size to this
YIELD_SIZE = 60 # seconds, biggest time yield
# making this smaller means the end time will be more accurate

# convenience functiont, to associate name with inflow queue
def infname(s):
    match s:
        case 1: return "A1"
        case 2: return "A2"
        case 3: return "B1"
        case 4: return "B2"
        case _: return "illegal"
# convenience function, to associate name with outflow queue   
def otfname(s):
    match s:
        case 5: return "A1"
        case 6: return "A2"
        case 7: return "B1"
        case 8: return "B2"
        case _: return "illegal"

# utility function for cartesian distance
def cart_distance(c1,c2):
    dx=c1[0]-c2[0]
    dy=c1[1]-c2[1]
    d_sq = dx*dx+dy*dy
    d = 0 if d_sq<=0 else math.sqrt( d_sq )
    return d

# utility function, perp distance point c3 to line c1-c2
# returns dist along c1-c2 to intersect and ppl distance
# used to check when a path gets close enough to a gate to
# sense its state and update the map
def ppl_distance(c1,c2,  c3):
    x1,y1=c1
    x2,y2=c2
    x3,y3=c3
    p1=np.array([x1,y1])
    p2=np.array([x2,y2])
    p3=np.array([x3,y3])
    if (p1==p2).all():
        p1 += (0.01,0.01)
    d = np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1)
    # perp distance, will check against sensor range for detection
    h =  np.linalg.norm(p1-p3)
    line_d_sq = h*h-d*d
    line_d = 0 if line_d_sq<=0 else math.sqrt( line_d_sq )
    # this is how far long the line you go to this sensor detection point
    full_d = np.linalg.norm(p2 - p1)
    r = line_d/full_d
    x4,y4 = x1 + (x2-x1)*r, y1 + (y2-y1)*r
    # this is the coordinates of that 'closest point'
    return d,line_d,(x4,y4) # per dist, dist along the line, closest pt

#
# Robot class
#
class Robot:
    # each robot has access to all outflow locs and to all gates for map update
    def __init__(self,env,name,loc,method,outlocs,gates):
        self.name=name
        self.location=loc
        self.state=STATE_READY
        self.method = method
        self.destination = None
        self.queue = None
        self.env = env
        self.outflow_locations=outlocs
        self.map = gt.Gates() # map is just the gate status, all open initially
        self.world = gates # the world is the actual gates
        self.last_gate = None # gate used last for crossing
        self.retries = ROBOT_FAIL_RETRIES # delivery fail? then retry this many times
        # metrics
        self.move_time = 0 # total time taken for acquire/deliver product
        self.move_dist = 0 # total distance moved acquire/deliver product
        self.num_products = 0 # total products, local and nonlocal
        # local means same side of facility and non local is other side
        self.num_nonlocal_products = 0 # total non local products 
        self.fails = 0 # how many 'give up' times, only nonlocal can fail
        self.replans = 0 # how many replanned paths
        # which run function is run de[ends on setting of method
        self.run = [ self.runMPP, self.runWAVN ] # the two functions
        #
        if DEBUG:
            print("Robot ",self.name,"@",self.location," started.")
        return
    
    # print out the current value of the robot metrics and its 'map'
    def metrics(self):
        print("Robot ",self.name," time ",self.env.now)
        print("Total time ",self.move_time)
        print("Total dist ",self.move_dist)
        print("Total prod (local,nonlocal)",self.num_products,self.num_nonlocal_products)
        print("Total fails ",self.fails)
        print("Total replan ",self.replans)
        self.map.display("Map")
        return

    # print out the final metrics values, including average (per product) values
    def final_metrics(self):
        p = self.num_products+self.num_nonlocal_products
        n = self.env.now
        print("---------------------------------")
        print(f"Robot {self.name} time {self.env.now:.2f} method {self.method}")
        print(f"Total work time {self.move_time:.2f} Av work time {self.move_time/p:.2f}")
        print(f"Total idle time {n-self.move_time:.2f} Av idle time {(n-self.move_time)/p:.2f}")
        print(f"Total dist {self.move_dist:.2f} Av dist {self.move_dist/p:.2f}")
        print(f"Total prod (local {self.num_products},nonlocal {self.num_nonlocal_products}),sum {p}")
        print(f"Total fails {self.fails}")
        print(f"Total replan {self.replans}")
        print("---------------------------------")
          
    # test if the destination is local or nonlocal
    def nonlocal_destination(self,destination):
        return (self.name==1 and destination[0]>50) or \
               (self.name==2 and destination[0]<50)

    # path plan function just produces straight line
    def pathplan(self,dest):
        xd = dest[0]-self.location[0]
        yd = dest[1] - self.location[1]
        distance = np.hypot( xd,yd)
        time = distance / ROBOT_SPEED + ROBOT_PP_TIME
        return distance,time

    # is any gate within sensor range of location? Update map if so
    def mapping(self, c1,c2):
        list_d=[]
        for i,c3 in enumerate(self.map.gate_location):
            d,line_d, pi=ppl_distance(c1,c2,  c3)
            list_d.append( (line_d,d,i) )
        y = sorted(list_d) # get the closest gate
        if y[0][1]<LIDAR_RANGE: # robot would sense this gate
                return y[0][0],y[0][2] # distance along the line and gate number
        return None,None

    # when is this gate within range of sensor
    def mapping_togate(self, c1,c2, g):
        c3 = self.map.gate_location[g]
        d,line_d,pi=ppl_distance(c1,c2,  c3)
        if d<LIDAR_RANGE: # robot would detect this gate
                return line_d,pi
        return None,None

    # check map and return list of which gates are open
    def check_map(self,themap):
        gate_list=[]
        for i,gs in enumerate(self.map.gate_status):
            if gs:
                gate_list.append(i)
        return gate_list

    # find the closest gate in the list that is open
    # if no gate is open, then randomly pick one and go there to explore
    # if it is open
    def closest_gate(self,gate_list):
        if len(gate_list)==0:
            return random.choice([0,1,2]) # explore strategy: just check to see if this is open
        d_list=[]
        for g in gate_list:
            d_list.append( (cart_distance(self.location,self.map.gate_enter[g]),g) )
        y = sorted(d_list)
        return y[0][1] # closest gate number

    # path plan a nonlocal path through a gate, 3 lines
    def gate_path(self,g): # path from location through gate to destination
        d = cart_distance(self.location,self.map.gate_enter[g] ) + \
             cart_distance(self.map.gate_enter[g],self.map.gate_exit[g])+ \
             cart_distance(self.map.gate_exit[g], self.destination)
        t = d/ROBOT_SPEED
        return d,t


    #
    # MPP motion strategy function
    #
    def runMPP(self,queue1,queue2,other_robot):
        while True:
            ts_s = self.move_time
            #-----------------------------------------------------
            # READY TO BE GET A PRODUCT from Q1 or Q2
            if self.state==STATE_READY and queue1.hasproduct( ): #Q1 product
                self.state = STATE_2INFLOW
                self.destination = queue1.location
                self.queue=queue1
                # plan the path for the qcuire
                d,t=self.pathplan(self.destination)
                # check if we can do mapping during the acquire
                sd,g = self.mapping(self.location,self.destination)
                if sd!=None: # Map can be updated
                    st = sd/ROBOT_SPEED # how soon till we get to the gate
                    yield self.env.timeout( st ) # allow time to elapse
                    if DEBUG:
                        print("R",self.name," mapping g=",g)
                    # do mapping: update the status of the gates now
                    self.map.gate_status[g]=self.world.gate_status[g]
                    other_robot.map.gate_status[g]=self.world.gate_status[g]
                else:
                    st=0 # no mapping update

                if DEBUG:
                    print("R",self.name,"->",infname(queue1.name)," pickup")
                    
                # break up the time yield into small chunks
                num_yields = int(t-st) // int(YIELD_SIZE)
                last_yield = (t-st) - num_yields*YIELD_SIZE
                for i in range(num_yields):
                    yield self.env.timeout( YIELD_SIZE)
                yield self.env.timeout(last_yield)
                # broken up yield completed
                
                self.move_time += t # assume mapping is in parallel with motion
                self.move_dist += d # update time and motion metrics
                
            #-----------------------------------------------------
            elif self.state==STATE_READY and queue2.hasproduct( ): # Q2 product
                self.state = STATE_2INFLOW
                self.destination = queue2.location
                self.queue=queue2
                d,t=self.pathplan(self.destination)
                sd,g = self.mapping(self.location,self.destination)
                if sd!=None: # can do mapping along this route
                    st = sd/ROBOT_SPEED # how soon till we get to the gate
                    yield self.env.timeout( st ) # allow time to elapse
                    if DEBUG:
                        print("R",self.name," mapping g=",g)
                    # do the mapping
                    self.map.gate_status[g]=self.world.gate_status[g]
                    other_robot.map.gate_status[g]=self.world.gate_status[g]
                else:
                    st=0 # no map update

                if DEBUG:
                    print("R",self.name,"->",infname(queue2.name)," pickup")
                    
                # break up the yields into smaller chunks
                num_yields = int(t-st) // int(YIELD_SIZE)
                last_yield = (t-st) - num_yields*YIELD_SIZE
                for i in range(num_yields):
                    yield self.env.timeout( YIELD_SIZE)
                yield self.env.timeout(last_yield)
                
                # update the robot metrics
                self.move_time += t
                self.move_dist += d
                
            #-----------------------------------------------------
            # PICKING UP THE PRODUCT FROM THE INFLOW Q
            elif self.state==STATE_2INFLOW: # arrived at inflow
                self.location = self.destination # at where you planned to be
                p = self.queue.pop() # get the product from the queue
                # and set a new destination
                self.destination = self.outflow_locations[p.product_destination]
                # check LOCAL or NONLOCAL path
                if self.nonlocal_destination(self.destination):
                    #
                    if DEBUG:
                        print("NONLOCAL DEST R",self.name," d=",self.destination," time=",self.env.now)
                        print("Robot map gates:")
                        self.map.display()
                    # Determine which gates the map says is open
                    open_gates = self.check_map(self.map) 
                    if DEBUG:
                        print("Open gates:",open_gates)
                    # Now look for a path through one of these gates
                    path_found=False
                    d,t=0,0  # total time and distance for this path, incl replans
                    pi=0  # closest approach to a gate for sensing
                    st=0  # total sensing time all path attempts
                    sti=0 # one path attempt sensing time
                    while not path_found and self.retries>0: # nonlocal PP loop
                        g = self.closest_gate(open_gates)
                        sd,pi = self.mapping_togate(self.location,self.map.gate_enter[g],g)
                        # was that gate open?
                        # either way, do mapping and update gate status
                        sti = sd/ROBOT_SPEED # how soon till we get to the gate
                        yield self.env.timeout( sti ) # allow time to elapse
                        # do the mapping
                        self.map.gate_status[g]=self.world.gate_status[g]
                        other_robot.map.gate_status[g]=self.world.gate_status[g]
                        # map updated
                        path_found = self.map.gate_status[g]
                        # if the gate was open, then the path is good
                        if DEBUG:
                            print("Gate status after approaching and upating by sensors ")
                            self.map.display()
                            print("World gates")
                            self.world.display()
                            print("Was this gate good? ",path_found)
                        if path_found: # just add the rest of the delivery route in
                            d1,t1 = self.gate_path(g) # 3 line path thru gate
                            d += d1 # added into whatever you did so far
                            st += sti # accumulate time yielded for sensing
                            self.last_gate = g # remember you went through here, for return
                        else: # this gate was closed, so set the location to be here and try again
                            if g in open_gates: # if map said it was open
                                open_gates.remove(g) # don't consider it
                            # make the robot location be the spot where it
                            #could see the status of the gate for mapping
                            self.location = pi # closest approach to this gate
                            if DEBUG:
                                print("gate was closed current d=",d," additiona sd=",sd)
                            d += sd # distance travelled to that closest point
                            st += sti # accumulate already yielded
                            self.replans += 1 # recard replan was needed
                        if len(open_gates)==0 and not path_found:
                            # need to explore and see if any gates have opened
                            if self.retries>0: # yes we can explore for now
                                self.map.gate_status=[True]*gt.NUM_GATES # assume all are open
                                self.retries -= 1 # and reduce available retries
                                if DEBUG:
                                    print("R",self.name," ** all gates blocked; allowing retry, left=",self.retries)
                                yield self.env.timeout(ROBOT_EXPLORE_WAIT) # allow a gate event
                    if not path_found: # tried all gates and possibly an explore
                        if DEBUG:
                            print("R",self.name," **pickup failed! all gates blocked.")
                            self.map.display()
                            self.world.display()
                        self.retries = ROBOT_FAIL_RETRIES # just give up
                        self.location = pi # last closest gate is give up position
                        self.fails += 1 # record this give up situation
                        self.state = STATE_READY # and be ready for next acquire
                    else: # path found
                        self.state = STATE_2OUTFLOW
                    # END of NONLOCAL product delivery
                else:
                    # LOCAL prouct delivery - easy case
                    # make a path to the destination
                    d,t=self.pathplan(self.destination)
                    # see if the map can be updated along the path
                    sd,g = self.mapping(self.location,self.destination)
                    if sd!=None: # Map can be updated
                        st = sd/ROBOT_SPEED # how soon till we get to the gate
                        yield self.env.timeout( st ) # allow time to elapse
                        if DEBUG:
                            print("R",self.name," mapping g=",g)
                        #do the mapping
                        self.map.gate_status[g]=self.world.gate_status[g]
                        other_robot.map.gate_status[g]=self.world.gate_status[g]
                        # mapping done
                    else:
                        st=0 # no mapping
                    self.state = STATE_2OUTFLOW
                    
                # here whether local/nonlocal pathfound/notfound
                t=d/ROBOT_SPEED # total time calculated by total distance
                if DEBUG:
                    if self.state == STATE_READY:
                        print("R",self.name,"->",otfname(p.product_destination)," dropoff aborted")
                    else:
                        print("R",self.name,"->",otfname(p.product_destination)," dropoff")
                        
                # break up total time into chunks
                # remove time already yielded for sensing
                num_yields = int(t-st) // int(YIELD_SIZE)
                last_yield = (t-st) - num_yields*YIELD_SIZE
                for i in range(num_yields):
                    yield self.env.timeout( YIELD_SIZE)
                yield self.env.timeout(last_yield)
                # done, broken into chunks
                
                self.move_time += t # update metrics
                self.move_dist += d
            #-----------------------------------------------------
            # DROPPING OFF THE PRODUCT TO OUTFLOW Q
            elif self.state==STATE_2OUTFLOW: # arrived for dropoff
                self.location = self.destination # got to where you were going
                # if this was a nonlocal delivery, will need to get back
                # local as well as drop off product
                if self.nonlocal_destination(self.destination): 
                    # NONLOCAL DROPOFF
                    # ASSUMPTION: just go back through the same gate you arrived
                    # harder case would be to actually check gate status and be stuck again
                    self.destination = self.map.gate_exit[ self.last_gate ] # assum still open
                    d,t = self.gate_path(self.last_gate) # back thru same gate
                    self.num_nonlocal_products += 1 # product delivered
                    self.last_gate = None # for safety
                    self.state=STATE_READY
                    # break up the time into smaller chunks
                    num_yields = int(ROBOT_DELIVER_TIME + t) // int(YIELD_SIZE)
                    last_yield = (ROBOT_DELIVER_TIME + t) - num_yields*YIELD_SIZE
                    for i in range(num_yields):
                        yield self.env.timeout( YIELD_SIZE)
                    yield self.env.timeout(last_yield)
                    # finished broken up deliver time
                    self.move_time += t # update metrics
                    self.move_dist += d
                else:
                    # easy case, LOCAL delivery
                    self.num_products += 1 # increment delivery metric
                    self.state=STATE_READY
                    yield self.env.timeout( ROBOT_DELIVER_TIME )
                if DEBUG:
                    print("R",self.name," delivered")
               
            #-----------------------------------------------------
            else: # this case if ready and no product yet
                yield self.env.timeout ( ROBOT_IDLE_TIME )


    #
    # WAVN motion strategy function
    #                  
    def runWAVN(self,queue1,queue2,other_robot):
        while True:
            #-----------------------------------------------------
            # READY TO PICKUP PRODUCT, Q1 or Q2
            if self.state==STATE_READY and queue1.hasproduct( ): # Q1
                self.state = STATE_2INFLOW
                self.destination = queue1.location
                self.queue=queue1
                d,t=self.pathplan(self.destination) # SL distance
                d += ROBOT_VH_DISTANCE              # VH penalty
                t += ROBOT_VH_DISTANCE/ROBOT_SPEED
                if DEBUG:
                    print("R",self.name,"->",infname(queue1.name)," pickup")
                yield self.env.timeout( t  ) # allow time to elapse
                self.move_time += t # update metrics
                self.move_dist += d
            #-----------------------------------------------------
            elif self.state==STATE_READY and queue2.hasproduct( ): # Q2
                self.state = STATE_2INFLOW
                self.destination = queue2.location
                self.queue=queue2
                d,t=self.pathplan(self.destination)
                d += ROBOT_VH_DISTANCE              # VH penalty
                t += ROBOT_VH_DISTANCE/ROBOT_SPEED
                if DEBUG:
                    print("R",self.name,"->",infname(queue2.name)," pickup")
                yield self.env.timeout( t  )
                self.move_time += t
                self.move_dist += d
            #-----------------------------------------------------
            # ACQUIRE produce from queue
            elif self.state==STATE_2INFLOW: # arrived at inflow
                self.location = self.destination
                p = self.queue.pop() # get the product
                self.destination = self.outflow_locations[p.product_destination]
                d,t=0,0
                lm_t = 0 # landmark time, set only if we did a common landmark search
                if self.nonlocal_destination(self.destination):
                    # NONLOCAL DELIVERY -- need common landmarks
                    if DEBUG:
                        print("NONLOCAL DEST R",self.name," d=",self.destination," time=",self.env.now)
                        print("World map gates:")
                        self.world.display()
                    # which gates are open, this are the ones you will see
                    # the common landmarks through, thus each gate status 
                    # tells if we can see that common landmark
                    open_gates = self.check_map(self.world)
                    yield self.env.timeout(ROBOT_LANDMARK_TIME) # time to do this
                    lm_t = ROBOT_LANDMARK_TIME
                    # now check and see which ones are visible
                    if DEBUG:
                        print("Open world gates = landmarks:",open_gates)
                    while len(open_gates)==0 and self.retries>0: # no landmark possible, so fail
                        yield self.env.timeout(ROBOT_EXPLORE_TIME)
                        self.retries -= 1
                        open_gates = self.check_map(self.world)
                        yield self.env.timeout(ROBOT_LANDMARK_TIME) # time to do this
                        lm_t += ROBOT_LANDMARK_TIME
                    if len(open_gates)!=0: # path found
                        # WAVN can't pick nearest - pick random gate to go through
                        landmark = random.choice(open_gates)
                        self.last_gate = landmark # same return assumption as MPP
                        d,t = self.gate_path(landmark)
                        d += ROBOT_VH_DISTANCE              # VH penalty
                        t += ROBOT_VH_DISTANCE/ROBOT_SPEED
                        self.state = STATE_2OUTFLOW
                    else: # no path found despite retries
                        self.fails += 1
                        self.retries = ROBOT_FAIL_RETRIES # reset 
                        self.status = STATE_READY
                else: # local travel                                              
                    d,t=self.pathplan(self.destination)
                    d += ROBOT_VH_DISTANCE              # VH penalty
                    t += ROBOT_VH_DISTANCE/ROBOT_SPEED
                    self.state = STATE_2OUTFLOW
                # here whether local/nonlocal pathfound/notfound
                if DEBUG:
                    print("R",self.name,"->",otfname(p.product_destination)," dropoff")

                yield self.env.timeout( t - lm_t )
                self.move_time += t
                self.move_dist += d
            #-----------------------------------------------------
            # DROPOFF the product at the outflow Q
            elif self.state==STATE_2OUTFLOW: # arrived for dropoff
                self.location = self.destination # at the destination you wanted
                # whats this a local or a nonlocal trip?
                if self.nonlocal_destination(self.destination):
                    # NONLOCAL
                    # have to get back local first
                    self.destination = self.map.gate_exit[ self.last_gate ] # assum still open
                    d,t = self.gate_path(self.last_gate)
                    d += ROBOT_VH_DISTANCE              # VH penalty
                    t += ROBOT_VH_DISTANCE/ROBOT_SPEED
                    self.num_nonlocal_products += 1
                    self.last_gate = None # for safety
                    self.state=STATE_READY
                    yield self.env.timeout( ROBOT_DELIVER_TIME + t )
                    self.move_time += t
                    self.move_dist += d
                else:
                    # LOCAL
                    self.num_products += 1
                    self.state=STATE_READY
                    yield self.env.timeout( ROBOT_DELIVER_TIME )
                if DEBUG:
                    print("R",self.name," delivered")

                
            #-----------------------------------------------------
            else: # this case if ready and no product yet
                yield self.env.timeout ( ROBOT_IDLE_TIME )
        



            
        
        
    
        
