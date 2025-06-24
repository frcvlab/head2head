#
# Head to head discrete event simulation
# compares mapping/path=planning with WAVN in warehouse application
#
# inflow queues
#
# D. Lyons June 2025
#
#
import random
import math
import simpy

DEBUG = True

#convenience function to associate qname with index
def qname(s):
    match s:
        case 1: return "A1"
        case 2: return "A2"
        case 3: return "B1"
        case 4: return "B2"
        case _: return "illegal"
#convenience function to associate qname with index        
def otfname(s):
    match s:
        case 5: return "A1"
        case 6: return "A2"
        case 7: return "B1"
        case 8: return "B2"
        case _: return "illegal"

#
# Inflow queue class definition
#
class Inflow:
    def __init__(self,name,loc,capacity):
        self.name = name
        self.location = loc       # location in space for inflow
        self.capacity = capacity  # how many items can it hold
        self.buffer=[]            # buffer to hold items
        if DEBUG:
            print("Flow ",self.name," cap=",self.capacity," started")
        return
    # check if an inflow queue has reached capacity
    def full(self):
        state = len(self.buffer)==self.capacity
        if state and DEBUG:
            print("q ",qname(self.name)," full")
        return state
    # check if an inflow queue has any product waiting in it
    def hasproduct(self):
        return len(self.buffer)>0
    # remove the oldest element in the inflow queue
    def pop(self):
        return self.buffer.pop(0)
    # add an element to the end of the inflow queue
    def push(self,p):
        if not self.full():
            self.buffer.append(p)
            if DEBUG:
                print("q",qname(self.name)," p",p.product_type,"->",
                  otfname(p.product_destination)," in |q|=",len(self.buffer),
                  "@",p.queue_time)
        # any product that overflowed is just lost, and we are not counting
        # it for now, but here is where it would be counted
        return
    
              
