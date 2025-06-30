#
# Head to head discrete event simulation
# compares mapping/path=planning with WAVN in warehouse application
#
# run an experiment for different gate_rate values for both motion
# strategies and then produces a graph(s) of comparison
#
# D. Lyons June 2025
#
#
import random
import math
import simpy

from matplotlib import pyplot as plt
import csv

#place to store the results as a CSV file
expfile="C:\\Users\\dmmly\\OneDrive\\Documents\\Damian Main\\Working Folders\\head2head\\exp.csv"

random.seed(42) # for repeatability testing

import product  as pr # class for the products
import inflow   as iq # class for the inflow product buffers
import robot    as rt # class for the robots
import gates    as gt # class for the gates/map/landmarks

# convenience definitions for time
SECONDS  = 1
MINUTES  = 60*SECONDS
HOURS    = 60*MINUTES

#*******************
SIM_TIME= 48*HOURS # how long the simulation will be for
#*******************
METHOD = rt.METHOD_MPP # just the default value

# debug flags for each module; turn off for experiments
DEBUG = False
pr.DEBUG = False
iq.DEBUG = False
rt.DEBUG = False
gt.DEBUG = False

# convenience function to associate name with inflow queue
def qname(s):
    match s:
        case 1: return "A1"
        case 2: return "A2"
        case 3: return "B1"
        case 4: return "B2"
        case _: return "illegal"
      

#locations in meters for inflow/outflow positions
# why are these not in the inflow module?

INFLOWA1_LOC = (0,65)
INFLOWA2_LOC = (0,35)

INFLOWB1_LOC = (100,65)
INFLOWB2_LOC = (100,35)

OUTFLOWA1_LOC = (20,100)
OUTFLOWA2_LOC = (20, 0)

OUTFLOWB1_LOC = (100-20,100)
OUTFLOWB2_LOC = (100-20,0)


# a diagnostic variable that can be used to check
# the status of each of the queues on a periodic basis
# not used in experiments

QCHECK_TIME = 400 # seconds

#
#
# Generator of the products
# This function makes products and places them in queues
#
def source(env,inflows):
    while True:
        p = pr.Product()
        p.make() # fills in the fields
        q = inflows[p.product_source-1]
        if not q.full():
            p.queue_time=env.now
            q.push(p)
            yield env.timeout(p.arrival_time)
        else:
            yield env.timeout(1) # in case of full qeueue
    return

#
#
# Generator of gate change/uncertainty
# This function randomly changes the gate status based on
# the gate_rate

def gate_change(env,gates):
    while True:
        change_time = random.expovariate(1.0/gt.GATE_RATE)
        change_gate = random.choices( list(range(gt.NUM_GATES)),
                                        gt.GATE_WEIGHTS )[0]
        yield env.timeout(change_time)
        gates.switch(change_gate)
    return

# If checking/debugging the queue status, this function will print
# q status periodically. Not used in experiments

def queuestate(env,queues):
    while True:
        for q in queues:
            print(qname(q.name)," size ",len(q.buffer))
        yield env.timeout( QCHECK_TIME )
    return

#
#
# FINAL METRICS CALCULATIONS
#



# This function takes the two robot metrics and then produces
# average metrics for time/distance and sum metrics for replanning, fails etc
# these combined robot metrics are what will be compared for both
# MPP and WAVN cases

def final_metrics(rlist,env):
    def mname(m):
        return "MPP" if m==0 else "WAVN" 
    r1,r2=rlist
    p1 = r1.num_products+r1.num_nonlocal_products
    p2 = r2.num_products+r2.num_nonlocal_products
    n = env.now
    move_time= 0.5*(r1.move_time+r2.move_time)
    av_move_time = 0.5*( r1.move_time/p1 + r2.move_time/p2)
    idle_time = n - move_time
    av_idle_time = 0.5*( (n-r1.move_time)/p1 + (n-r2.move_time)/p2)
    move_dist = 0.5*(r1.move_dist + r2.move_dist)
    av_move_dist = 0.5*( r1.move_dist/p1 + r2.move_dist/p2)
    av_staleness = 0.5*(r1.total_staleness+r2.total_staleness)
    if DEBUG:
        r1.final_metrics()
        r2.final_metrics()
        print("*******************************")
        print(f"Method {mname(r1.method)}")
        print(f"Total work time {move_time:.2f} Av work time {av_move_time:.2f}")
        print(f"Total idle time {idle_time:.2f} Av idle time {av_idle_time:.2f}")
        print(f"Total dist {move_dist:.2f} Av dist {av_move_dist:.2f}")
        print(f"Total prod {p1+p2}")
        print(f"Total fails {r1.fails+r2.fails}")
        print(f"Total replan {r1.replans+r2.replans}")
        print(f"Total av staleness {av_staleness}")
        print("*******************************")
        print
    # return all these calculate final metrics
    return move_time, av_move_time, idle_time, \
           av_idle_time, move_dist,av_move_dist,\
           p1+p2,r1.replans,av_staleness

# names for the metrics returned from the final_metrics function
final_metric_list=["Total work time","Av work time","Total idle time","Av idle time",\
                   "Total dist","Av dist","Total prod", "Num Replans","Av Staleness"]

#move_time, av_move_time, idle_time, av_idle_time, move_dist,av_move_dist,p1+p2,replans,staleness
# 0           1             2           3              4          5        6     7        8
 
#
#
# Run experiment
#
# This function will run one head to head experiment
#
def do_h2h_experiment():
    #
    res=[]
    # each method in turn is evaluated
    for METHOD in [rt.METHOD_MPP, rt.METHOD_WAVN]:
        #
        env = simpy.Environment()
        # make inflow queues
        INFLOW_QUEUES = [ iq.Inflow(pr.INFLOWA1, INFLOWA1_LOC, 4),
                          iq.Inflow(pr.INFLOWA2, INFLOWA2_LOC, 4),
                          iq.Inflow(pr.INFLOWB1, INFLOWB1_LOC, 4),
                          iq.Inflow(pr.INFLOWB2, INFLOWB2_LOC, 4) ]
        #make outflow queues
        OUTFLOW_QUEUES = {pr.OUTFLOWA1:OUTFLOWA1_LOC, pr.OUTFLOWA2:OUTFLOWA2_LOC,
                          pr.OUTFLOWB1:OUTFLOWB1_LOC, pr.OUTFLOWB2:OUTFLOWB2_LOC } 
        
        #gates between the parts of the facility
        GATES = gt.Gates() # all start Open

        #make the robots
        ROBOTS = [ rt.Robot(env,rt.ROBOTA, rt.LOCATIONA, METHOD, OUTFLOW_QUEUES,GATES),
                   rt.Robot(env,rt.ROBOTB, rt.LOCATIONB, METHOD, OUTFLOW_QUEUES,GATES)
                   ]

        # make the generator of products
        env.process( source(env,INFLOW_QUEUES) )

        # only allow this for monitoring/debugging queue status
        #env.process( queuestate(env,INFLOW_QUEUES) )

        # make the generator of gate changes
        env.process( gate_change(env,GATES) )

        # add the two robots
        env.process( ROBOTS[0].run[METHOD](INFLOW_QUEUES[0], INFLOW_QUEUES[1], ROBOTS[1]) )
        env.process( ROBOTS[1].run[METHOD](INFLOW_QUEUES[2], INFLOW_QUEUES[3], ROBOTS[0]) )

        #now RUN the simulation until SIM_TIME
        env.run(until=SIM_TIME)
        #for r in ROBOTS:
        #    r.final_metrics()

        # record the results
        res.append( final_metrics(ROBOTS,env))

    #unpack results for the MPP and WAVN cases
    res1,res2=res

    #calculate the ratio of one to the other, for DEBUG purposes
    print("Comparing (MPP-WAVN)/MPP for each metric; -ve=> WAVN bigger.")
    for i,(x,y) in enumerate(zip(res1,res2)):
        if y!=0:
            print(f"{final_metric_list[i]} {ratio(x,y):.2f}%")
        else: # replans
            print(f"{final_metric_list[i]} {x:.2f}")

    print("Done.")
    return res

#convenience function to calculate the comparison ration
def ratio(x,y):
    return (100 * (x-y)/x) if x!=0 else 0

#------MAIN PROGRAM-------------

# what gates rates to run in the experiment
gate_rates =  list( range(1,12*60,15) )#  +list( range(2*60,48*60,60) )

#gate_rates =[ 1 ]

# list of metrics to be written out
products_made=[]
idle_time = []
replans=[]
staleness=[]
move_time=[]

# how many tims to rerun a simulation to get a statistically accurate average
num_samples = 50

# how many results are returned from the final_metrics function
num_results = 9 


# RUN EXPERIMENT
for gr in gate_rates: # eveluate for each gate rate

    gt.GATE_RATE = gr * MINUTES
    print("rate=",gt.GATE_RATE)

    # variables to record results of num_samples identical experiments
    sumres_mpp,sumres_wavn=[],[]
    for n in range(num_samples): # n samples of same experiment
        print("sample ",n)
        res = do_h2h_experiment() # do one head to head experiment
        
        while res[0][0]<SIM_TIME*0.7: # check for overly short experiment
            print("repeat ",n,res[0][0]) # eliminates outliers
            res = do_h2h_experiment()
            
        res_n_mpp,res_n_wavn=res
        sumres_mpp.append( res_n_mpp )
        sumres_wavn.append( res_n_wavn )

    # variables to claculate averages of num_samples identical experiments
    res_mpp,res_wavn=[0]*num_results,[0]*num_results
    # calculate the averages
    for i in range(num_results): # average over n
        res_mpp[i] = sum( [ sumres_mpp[k][i] for k in range(num_samples) ] )/num_samples
        res_wavn[i] = sum( [ sumres_wavn[k][i] for k in range(num_samples) ] )/num_samples

    #add the average results to the metrics lists
    products_made.append( ratio(res_mpp[6],res_wavn[6]))
    idle_time.append( ratio(res_mpp[3],res_wavn[3]) )
    move_time.append( ratio(res_mpp[1],res_wavn[1]) )
    replans.append( res_mpp[7] )
    staleness.append( res_mpp[8] )

# All experiments completed
# Write out the results as a CSV file
# and also use matplot lib to show what the results look like
with open(expfile,"w") as csvfile:
    csvw = csv.writer(csvfile)
    csvw.writerow(["GR"]+gate_rates)
    csvw.writerow(["RP"]+replans)
    plt.plot(gate_rates,replans)
    plt.show()
    csvw.writerow(["ST"]+staleness)
    plt.plot(gate_rates,staleness)
    plt.show()
    csvw.writerow(["PM"]+products_made)
    plt.plot(gate_rates, products_made)
    plt.show()
    csvw.writerow(["MT"]+move_time)
    plt.plot(gate_rates,move_time)
    plt.show()
    csvw.writerow(["IT"]+idle_time)
    plt.plot(gate_rates,idle_time)
    plt.show()
#---------------------------------END OF PROGRAM-------------------------
