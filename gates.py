#
# Head to head discrete event simulation
# compares mapping/path=planning with WAVN in warehouse application
#
# Gates and map
#
# D. Lyons June 2025
#
#
import random
import math
import simpy

DEBUG = True

NUM_GATES = 3       # how many gates, routes through the center of the facility

# locations of the gates
GATE1_LOC = (50,95) # meters
GATE2_LOC = (50,50)
GATE3_LOC = (50, 5)

# to go through a gate, a robot moves thru the entrance->exit in a line
GATE_ENTRANCE = (-5,0) # alleyway through the gate
GATE_EXIT     = ( 5,0)

# convenience constant
MINUTES = 60

# exponential distribution rate for the status change of gates
# default value
GATE_RATE = 10*MINUTES # seconds

GATE_WEIGHTS = [1,1,1] # all equally likely to change state

# convenience function to help setup gate locations
def addc(c1,c2):
    return ( c1[0]-c2[0], c1[1]-c2[1] )

#
# Class definition for the gates (which is the same as a map)
# since the map only contains a robot's local repr of gate status observed
#
class Gates:
    def __init__(self):
        self.gate_status = [True] * NUM_GATES # True==open
        self.gate_location = [GATE1_LOC, GATE2_LOC, GATE3_LOC]
        self.gate_enter = [ addc( GATE1_LOC, GATE_ENTRANCE ),
                       addc( GATE2_LOC, GATE_ENTRANCE ),
                       addc( GATE3_LOC, GATE_ENTRANCE ) ]
        self.gate_exit  = [ addc( GATE1_LOC, GATE_EXIT ),
                       addc( GATE2_LOC, GATE_EXIT ),
                       addc( GATE3_LOC, GATE_EXIT ) ]
        self.update_time = [0.0]*NUM_GATES # when was this gate last updated
        return
    # mark a gate as open
    def open(self,g):
        if g<0 or g>= NUM_GATES:
            return
        else:
            self.gate_status[g]=True
        return
    # mark a gate as closed
    def close(self,g):
        if g<0 or g>= NUM_GATES:
            return
        else:
            self.gate_status[g]=False
        return
    # switch the status for a gate from open->closed or closed->open
    def switch(self,g):
        if g<0 or g>= NUM_GATES:
            return
        else:
            self.gate_status[g]=not self.gate_status[g]
            if DEBUG:
                print("***Gate ",g,", changing state TO ",self.gate_status[g])
                self.display()
        return
    # display the status of the gates
    # s can be used to customize whether this is the world, or a robot map
    def display(self,s="Gates"):
        print(s+" ",end='')
        for g in range(NUM_GATES):
            print(g,":",self.gate_status[g],end=" ")
        print(".")
        return
    # check is gate g open?
    def status(self,g):
        if g<0 or g>= NUM_GATES:
            return None
        else:
            return self.gate_status[g]

    
    
