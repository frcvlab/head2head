#
# Head to head discrete event simulation
# compares mapping/path=planning with WAVN in warehouse application
#
# D. Lyons June 2025
#
#
import random
import math
import simpy

random.seed(42) # for repeatability testing

import product  as pr # class for the products
import inflow   as iq # class for the inflow product buffers
import robot    as rt # class for the robots
import gates    as gt # class for the gates/map/landmarks

SECONDS  = 1
MINUTES  = 60*SECONDS
HOURS    = 60*MINUTES

SIM_TIME= 48*HOURS
METHOD = rt.METHOD_MPP

pr.DEBUG = False
iq.DEBUG = False
rt.DEBUG = False
gt.DEBUG = False


def qname(s):
    match s:
        case 1: return "A1"
        case 2: return "A2"
        case 3: return "B1"
        case 4: return "B2"
        case _: return "illegal"
      

#measurements in meters for inflow/outflow positions
INFLOWA1_LOC = (0,65)
INFLOWA2_LOC = (0,35)

INFLOWB1_LOC = (100,65)
INFLOWB2_LOC = (100,35)

OUTFLOWA1_LOC = (20,100)
OUTFLOWA2_LOC = (20, 0)

OUTFLOWB1_LOC = (100-20,100)
OUTFLOWB2_LOC = (100-20,0)



QCHECK_TIME = 400 # seconds


#generator of the products

def source(env,inflows):
    while True:
        p = pr.Product()
        p.make() # fills in the fields
        q = inflows[p.product_source-1]
        if not q.full():
            p.queue_time=env.now
            q.push(p)
            yield env.timeout(p.arrival_time)
        yield env.timeout(1) # in case of saturation
    return

# generator of gate change/uncertainty

def gate_change(env,gates):
    while True:
        change_time = random.expovariate(1.0/gt.GATE_RATE)
        change_gate = random.choices( list(range(gt.NUM_GATES)),
                                        gt.GATE_WEIGHTS )[0]
        yield env.timeout(change_time)
        gates.switch(change_gate)
    return

#-----------------------------
def queuestate(env,queues):
    while True:
        for q in queues:
            print(qname(q.name)," size ",len(q.buffer))
        yield env.timeout( QCHECK_TIME )
    return



final_metric_list=["Total work time","Av work time","Total idle time","Av idle time",\
                   "Total dist","Av dist","Total prod"]
def final_metrics(rlist):
    def mname(m):
        return "MPP" if m==0 else "WAVN" 
    r1,r2=rlist
    p1 = r1.num_products+r1.num_nonlocal_products-r1.fails
    p2 = r2.num_products+r2.num_nonlocal_products-r2.fails
    n = env.now
    move_time= 0.5*(r1.move_time+r2.move_time)
    av_move_time = 0.5*( r1.move_time/p1 + r2.move_time/p2)
    idle_time = n - move_time
    av_idle_time = 0.5*( (n-r1.move_time)/p1 + (n-r2.move_time)/p2)
    move_dist = 0.5*(r1.move_dist + r2.move_dist)
    av_move_dist = 0.5*( r1.move_dist/p1 + r2.move_dist/p2)
    print("*******************************")
    print(f"Method {mname(r1.method)}")
    print(f"Total work time {move_time:.2f} Av work time {av_move_time:.2f}")
    print(f"Total idle time {idle_time:.2f} Av idle time {av_idle_time:.2f}")
    print(f"Total dist {move_dist:.2f} Av dist {av_move_dist:.2f}")
    print(f"Total prod {p1+p2}")
    print(f"Total fails {r1.fails+r2.fails}")
    print(f"Total replan {r1.replans+r2.replans}")
    print("*******************************")
    print
    return move_time, av_move_time, idle_time, av_idle_time, move_dist,av_move_dist,p1+p2
# make the inflows

res=[]
for METHOD in [rt.METHOD_MPP, rt.METHOD_WAVN]:

    env = simpy.Environment()
    
    INFLOW_QUEUES = [ iq.Inflow(pr.INFLOWA1, INFLOWA1_LOC, 4),
                      iq.Inflow(pr.INFLOWA2, INFLOWA2_LOC, 4),
                      iq.Inflow(pr.INFLOWB1, INFLOWB1_LOC, 4),
                      iq.Inflow(pr.INFLOWB2, INFLOWB2_LOC, 4) ]

    OUTFLOW_QUEUES = {pr.OUTFLOWA1:OUTFLOWA1_LOC, pr.OUTFLOWA2:OUTFLOWA2_LOC,
                      pr.OUTFLOWB1:OUTFLOWB1_LOC, pr.OUTFLOWB2:OUTFLOWB2_LOC } 

    #gates between the parts of the facility

    GATES = gt.Gates() # all start Open

    #make the robots



    ROBOTS = [ rt.Robot(env,rt.ROBOTA, rt.LOCATIONA, METHOD, OUTFLOW_QUEUES,GATES),
               rt.Robot(env,rt.ROBOTB, rt.LOCATIONB, METHOD, OUTFLOW_QUEUES,GATES)
               ]



    env.process( source(env,INFLOW_QUEUES) )

    #env.process( queuestate(env,INFLOW_QUEUES) )

    env.process( gate_change(env,GATES) )

    env.process( ROBOTS[0].run[METHOD](INFLOW_QUEUES[0], INFLOW_QUEUES[1], ROBOTS[1]) )
    env.process( ROBOTS[1].run[METHOD](INFLOW_QUEUES[2], INFLOW_QUEUES[3], ROBOTS[0]) )

    env.run(until=SIM_TIME)

    #for r in ROBOTS:
    #    r.final_metrics()
    res.append( final_metrics(ROBOTS))

res1,res2=res
print("Comparing (MPP-WAVN)/MPP for each metric; -ve=> WAVN bigger.")
for i,(x,y) in enumerate(zip(res1,res2)):
    print(f"{final_metric_list[i]} {100*(x-y)/x:.2f}%")
print("Done.")
           
