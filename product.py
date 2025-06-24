#
# Head to head discrete event simulation
# compares mapping/path=planning with WAVN in warehouse application
#
# definition class of the products that arrive
#
# D. Lyons June 2025
#
#
import random
import math
import simpy
#

DEBUG = True

#indices and parameters for product types

# at the moment product type is not being used, only
# product inflow location and outflow location

TYPEA = 0
TYPEB = 1
TYPES = [TYPEA, TYPEB]
TYPEW = [50,50] # probability distribution of types

#indices and parameters for the inflow/outflow
# Inflow quests for the A side of the facility
INFLOWA1 = 1
INFLOWA2 = 2
INFLOWA  = [INFLOWA1,INFLOWA2]
INFLOWAW = [50, 50] # probability distribution for inflow events

#Inflow queues for the B side of the facility
INFLOWB1 = 3
INFLOWB2 = 4
INFLOWB  = [INFLOWB1,INFLOWB2]
INFLOWBW = [50, 50] # probability distribution for inflow events

#indices for the A and B side outflow queues
OUTFLOWA1 = 5
OUTFLOWA2 = 6
OUTFLOWB1 = 7
OUTFLOWB2 = 8

# probability distribution for outflow destination on A side
OUTFLOWA  = [ OUTFLOWA1, OUTFLOWA2, OUTFLOWB1, OUTFLOWB2]
OUTFLOWAW = [ 30, 30, 20, 20 ]

# probability distribution for outflow destination on B side
OUTFLOWB  = [ OUTFLOWB1, OUTFLOWB2, OUTFLOWA1, OUTFLOWA2]
OUTFLOWBW = [ 30, 30, 20, 20 ]

# The exponential distribution rates for all the inflow queues
INFLOW_RATE = [ 1.0/100, 1.0/200, 1.0/200, 1.0/100 ]

# convenience function to associate qname with index
def infname(s):
    match s:
        case 1: return "A1"
        case 2: return "A2"
        case 3: return "B1"
        case 4: return "B2"
        case _: return "illegal"
# convenience function to associate qname with index
def otfname(s):
    match s:
        case 5: return "A1"
        case 6: return "A2"
        case 7: return "B1"
        case 8: return "B2"
        case _: return "illegal"   
#
# Product class definition
#
class Product:
    def __init__(self):
        self.product_type = None        # not being used for now
        self.product_source = None      # which inflow
        self.product_destination = None #which outflow
        self.arrival_time = None        # exp arrival time
        self.queue_time=None            # time it entered the queue
    # function to generate a product
    def make(self):
        self.product_type = random.choices(TYPES,TYPEW)[0]
        if self.product_type==TYPEA:
            self.product_source = random.choices(INFLOWA,INFLOWAW)[0]
            self.product_destination = random.choices(OUTFLOWA,OUTFLOWAW)[0]
        else:
            self.product_source = random.choices(INFLOWB,INFLOWBW)[0]
            self.product_destination = random.choices(OUTFLOWB,OUTFLOWBW)[0]
        rate=INFLOW_RATE[self.product_source-1]
        self.arrival_time = random.expovariate(rate)
        if DEBUG:
            print("P, typ=",self.product_type, " s=",infname(self.product_source),
              " d=",otfname(self.product_destination), ", t=",self.arrival_time)
    
    
