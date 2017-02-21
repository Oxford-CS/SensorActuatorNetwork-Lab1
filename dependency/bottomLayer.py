# Network simulator for routing and transport protocols,
# This file defines basic objects to establish a network: 1. Node 2. Link 3. Packet

import random, sys, wx, math

################################################################################
#
# Node -- a network node
#
# Node.reset()        -- reset node's state at start of simulation
# Node.add_packet(p)  -- add packet to node's transmit queue
# Node.receive(p)     -- called to process packet sent to this node
# Node.transmit(time) -- allow node to send packets at current time
# Node.forward(p)     -- lookup route for pkt p and send it on appropriate link
# Node.arrived_on(p)  -- returns link that packet p just arrived on
#
################################################################################

class Node:
    def __init__(self,location,address=None):
        self.location = location
        if address is None: self.address = location
        else: self.address = address
        self.links = []  # links that connect to this node
        self.packets = []  # packets to be processed this timestep
        self.transmit_queue = []  # packets to be transmitted from this node
        self.receive_queue = []  # packets received by this node
        self.properties = {}
        self.network = None  # will be filled in later
        self.nsize = 0       # filled in by draw method

    def __repr__(self):
        return 'Node<%s>' % str(self.address)

    # reset to initial state
    def reset(self):
        for l in self.links: l.reset()
        self.transmit_queue = []   # nothing to transmit
        self.receive_queue = []    # nothing received
        self.queue_length_sum = 0  # reset queue statistics
        self.queue_length_max = 0
	self.neighbors.clear()
        self.routes.clear()
        self.routes[self.address] = 'Self'
        self.properties.clear()

    # keep track of links that connect to this node
    def add_link(self,l):
        self.links.append(l)

    # add a packet to be transmitted from this node.  Transmit queue
    # is kept ordered by packet start time.
    def add_packet(self,p):
        index = 0
        for pp in self.transmit_queue:
            if p.start < pp.start:
                self.transmit_queue.insert(index,p)
                break
            else: index += 1
        else: self.transmit_queue.append(p)

    # first phase of simulation timestep: collect one packet from
    # each incoming link
    def phase1(self):
        self.packets = [link.receive(self) for link in self.links]

    # second phase of simulation timestep: process arriving packets
    def phase2(self,time):
        # process each arriving packet
        for link_p in self.packets:
            if link_p is not None: self.process(link_p[1],link_p[0],time)
        self.packets = []

        # give this node a chance to transmit some packets
        self.transmit(time)

        # compute number of packets this node has queued up on its
        # outgoing links.  So we can compute queue length stats, keep
        # track of max and sum.
        pending = 0
        for link in self.links: pending += link.queue_length(self)
        self.queue_length_sum += pending
        self.queue_length_max = max(self.queue_length_max,pending)

        # report total number of packets that need processing
        return pending + len(self.transmit_queue)

    # default processing for packets addressed to this node -- just
    # keep a list of them
    def receive(self,p,link):
        self.receive_queue.append(p)

    # called each simulation cycle to give this node a chance to send
    # some packets.  Default behavior: source packets from a transmit
    # queue based on packets' specified start time.
    def transmit(self,time):
        # look for packets on this node's transmit queue whose time has come
        while len(self.transmit_queue) > 0:
            if self.transmit_queue[0].start <= time:
                self.process(self.transmit_queue.pop(0),None,time)
            else: break

    # OVERRIDE: forward packet onto proper outgoing link.  Default behavior
    # is to pick a link at random!
    def forward(self,p):
        link = random.choice(self.links)
        link.send(self,p)

    # deal with each packet arriving at or sent from this node
    def process(self,p,link,time):
        if p.destination == self.address:
            # it's for us!  Just note time of arrival and pass it receive
            p.finish = time
            self.receive(p,link)
        else:
            p.add_hop(self,time)
            self.forward(p)

    #########################################################
    # support for graphical simulation interface
    #########################################################

    # draw ourselves on the screen as a colored square with black border
    def draw(self,dc,transform):
        self.nsize = transform[0]/16
        loc = self.net2screen(transform)
        dc.SetPen(wx.Pen('black',1,wx.SOLID))
        dc.SetBrush(wx.Brush(self.properties.get('color','black')))
        dc.DrawRectangle(loc[0]-self.nsize,loc[1]-self.nsize,
                         2*self.nsize+1,2*self.nsize+1)
        label = str(self.address)
        dc.SetTextForeground('light grey')
        dc.SetFont(wx.Font(max(4,self.nsize*2),wx.SWISS,wx.NORMAL,wx.NORMAL))
        dc.DrawText(label,loc[0]+self.nsize+2,loc[1]+self.nsize+2)

        if len(self.transmit_queue) > 0:
            self.transmit_queue[0].draw(dc,transform,
                                        loc[0]-2*self.nsize,loc[1]-2*self.nsize)

    # if pos is near us, return status string
    def nearby(self,pos):
        dx = self.location[0] - pos[0]
        dy = self.location[1] - pos[1]
        if abs(dx) < .1 and abs(dy) < .1:
            return self.status()
        elif len(self.transmit_queue) > 0:
            if (dx > .1 and dx < .2) and (dy > .1 and dy < .2):
                return 'Unsent '+self.transmit_queue[0].status()
        else:
            return None

    def click(self,pos,which):
        dx = self.location[0] - pos[0]
        dy = self.location[1] - pos[1]
        if abs(dx) < .1 and abs(dy) < .1:
            self.OnClick(which)
            return True
        else:
            return False

    def OnClick(self,which):
        pass

    # status report to appear in status bar when pointer is nearby
    def status(self):
        return self.__repr__()

################################################################################
#
# Link -- a communication link between two network nodes
#
# Link.queue_length(n) -- count undelivered packets sent by specified node
# Link.other_end(n)    -- return node at other end of link
# Link.receive(n)      -- return one packet destined for specified node (or None)
# Link.send(n,p)       -- send packet to other end of link
#
################################################################################
class Link:
    def __init__(self,n1,n2):
        self.end1 = n1   # node at one end of the link
        self.end2 = n2   # node at the other end
        self.q12 = []    # queue of packets to be delivered to end2
        self.q21 = []    # queue of packets to be delivered to end1
        self.cost = 1    # by default, cost is 1
        self.costrepr = str(self.cost) # representing cost in GUI
        self.network = None  # will be filled in later
        n1.add_link(self)
        n2.add_link(self)
        self.broken = False

    def __repr__(self):
        return 'link(%s<-->%s) (%.1f)' % (self.end1,self.end2, self.cost)

    def reset(self):
        self.q12 = []    # reset packet queues
        self.q21 = []

    # return count of undelivered packets sent by specified node
    def queue_length(self,n):
        if n == self.end1: return len(self.q12)
        elif n == self.end2: return len(self.q21)
        else: raise Exception,'bad node in Link.queue_length'

    # return (link, packet) destined for specified node (or None)
    def receive(self,n):
        if n == self.end1:
            if len(self.q21) > 0: return (self, self.q21.pop(0))
            else: return None
        elif n == self.end2:
            if len(self.q12): return (self, self.q12.pop(0))
            else: return None
        else: raise Exception,'bad node in Link.receive'

    # send one packet from specified node
    def send(self,n,p):
        if self.broken: return
        if n == self.end1: self.q12.append(p)
        elif n == self.end2: self.q21.append(p)
        else: raise Exception,'bad node in Link.send'

######################################################################
"""A link with cost (higher cost means worse link)
"""
######################################################################
class CostLink(Link):
    def __init__(self,n1,n2):
        Link.__init__(self,n1,n2)
        self.nsize = 0                # filled in by draw method
        loc1 = n1.location
        loc2 = n2.location
        dx2 = (loc1[0] - loc2[0])*(loc1[0] - loc2[0])
        dy2 = (loc1[1] - loc2[1])*(loc1[1] - loc2[1])
        self.cost = math.sqrt(dx2 + dy2)
#        self.cost = random.randint(1,10)
        if (int(self.cost) == self.cost):
            self.costrepr = str(self.cost)
        else:
            self.costrepr = "sqrt(" + str(dx2+dy2) + ")"

    # method to set the cost of a link
    def set_cost(self, cost):
        self.cost = cost

class LossyCostLink(CostLink):
    def __init__(self,n1,n2,lossprob):
        CostLink.__init__(self,n1,n2)
        self.lossprob = lossprob
        self.linkloss = 0       # number of pkts lost on link

    def send(self,n,p):
        # we lose packets with probability self.lossprob
        if random.random() > self.lossprob:
            CostLink.send(self,n,p)
        else:
            self.linkloss = self.linkloss + 1 # stats on number of losses

################################################################################
#
# Packet -- data to be sent from one network node to another
#
# Packet.arrived_from() -- return node this packet just arrived from
#
################################################################################
class Packet:
    def __init__(self,src,dest,type,start,**props):
        self.source = src     # address of node that originated packet
        self.destination = dest  # address of node that should receive packet
        self.type = type
        self.start = start # simulation time at which packet was transmitted
        self.finish = None # simulation time at which packet was received
        self.route = []    # list of nodes this packet has visited
        self.network = None     # will be filled in later
        self.properties = props.copy()

    def __repr__(self):
        return 'Packet<%s to %s> type %s' % (self.source,self.destination,self.type)

    # keep track of where we've been
    def add_hop(self,n,time):
        self.route.append((n,time))

    #########################################################
    # support for graphical simulation interface
    #########################################################

    def draw(self,dc,transform,px,py):
        c = self.properties.get('color','blue')
        dc.SetPen(wx.Pen(c,1,wx.SOLID))
        dc.SetBrush(wx.Brush(c))
        radius = transform[0]/16
        dc.DrawCircle(px,py,radius)

    def draw_on_link(self,dc,transform,n1,n2):
        px = n1[0] + int(0.2*(n2[0] - n1[0]))
        py = n1[1] + int(0.2*(n2[1] - n1[1]))
        self.draw(dc,transform,px,py)

    def nearby(self,pos,n1,n2):
        px = n1[0] + 0.2*(n2[0] - n1[0])
        py = n1[1] + 0.2*(n2[1] - n1[1])
        dx = px - pos[0]
        dy = py - pos[1]
        if abs(dx) < .1 and abs(dy) < .1:
            return self.status()
        else: return None

    def status(self):
        return self.__repr__()

