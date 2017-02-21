# Network simulator for routing and transport protocols,
# This file defines top-layer objects, network, router, randomGraph

from bottomLayer import *
import scipy.spatial.distance as sci_dist
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from pass_loss_model import *

################################################################################
#
# Network -- a collection of network nodes, links and packets
#
# Network.make_node(loc,address=None)  -- make a new network node
# Network.add_node(x,y,address=None)   -- add a new node at specified location
# Network.find_node(x,y)               -- return node at given location
# Network.map_node(f,default=0)        -- see below
# Network.make_link(n1,n2)             -- make a new link between n1 and n2
# Network.add_link(x1,y2,x2,y2)        -- add link between specified nodes
#
# Network.make_packet(src,dst,type,start,**props)  -- make a new packet
# Network.duplicate_packet(p)          -- duplicate a packet
#
# Network.reset()                      -- initialize network state
# Network.step(count=1)                -- simulate count timesteps
#
################################################################################
class Network:
    def __init__(self,simtime):
        self.nodes = {}
        self.addresses = {}
        self.nlist = []
        self.links = []
        self.time = 0
        self.pending = 0
        self.packets = []
        self.npackets = 0
        self.max_x = 0
        self.max_y = 0
        self.simtime = simtime
        self.playstep = 1.0     # 1 second play step by default

        self.numnodes = 0       # TBD

    # override to make your own type of node
    def make_node(self,loc,address=None):
        return Node(loc,address=address)

    # add a node to the network
    def add_node(self,x,y,address=None):
        n = self.find_node(x,y)
        if n is None:
            n = self.make_node((x,y),address=address)
            n.network = self
            if address is not None:
                self.addresses[address] = n
            self.nlist.append(n)
            ynodes = self.nodes.get(x,{})
            ynodes[y] = n
            self.nodes[x] = ynodes
            self.max_x = max(self.max_x,x)
            self.max_y = max(self.max_y,y)
        return n

    def set_nodes(self,n):
        self.numnodes = n

    # locate a node given its location
    def find_node(self,x,y):
        ynodes = self.nodes.get(x,None)
        if ynodes is not None:
            return ynodes.get(y,None)
        return None

    # apply f to each network node in top-to-bottom, left-to-right
    # order.  Returns list of return values (default value is used
    # if a particular grid point doesn't contain a node).  Useful
    # for gathering statistical data that can be processed by Matlab.
    def map_node(self,f,default=0):
        result = []
        for row in xrange(self.max_y+1):
            for col in xrange(self.max_x+1):
                node = self.find_node(row,col)
                if node: result.append(f(node))
                else: result.append(default)
        return result

    # override to make your own type of link
    def make_link(self,n1,n2):
        return Link(n1,n2)

    # add a link between nodes at the specified locations
    def add_link(self,x1,y1,x2,y2):
        n1 = self.find_node(x1,y1)
        n2 = self.find_node(x2,y2)
        if n1 is not None and n2 is not None:
            link = self.make_link(n1,n2)
            link.network = self
            self.links.append(link)

    # override to make your own type of packet
    def make_packet(self,src,dest,type,start,**props):
        p = Packet(src,dest,type,start,**props)
        p.network = self
        self.packets.append(p)
        self.npackets += 1
        return p

    # duplicate existing packet
    def duplicate_packet(self,old):
        return self.make_packet(old.source,old.destination,old.type,self.time,
                                **old.properties)

    # compute manhattan distance between two nodes
    def manhattan_distance(self,n1,n2):
        dx = n1[0] - n2[0]
        dy = n1[1] - n2[1]
        return abs(dx) + abs(dy)

    # return network to initial state
    def reset(self):
        for n in self.nlist: n.reset()
        self.time = 0
        self.pending = 0
        self.packets = []
        self.npackets = 0
        self.pending = 1    # ensure at least simulation step

    # simulate network one timestep at a time.  At each timestep
    # each node processes one packet from each of its incoming links
    def step(self,count=1):
        stop_time = self.time + count
        while self.time < stop_time and self.pending > 0:
            # phase 1: nodes collect one packet from each link
            for n in self.nlist: n.phase1()

            # phase 2: nodes process collected packets, perhaps sending
            # some to outgoing links.  Also nodes can originate packets
            # of their own.
            self.pending = 0
            for n in self.nlist: self.pending += n.phase2(self.time)

            # increment time
            self.time += 1
        return self.pending

################################################################################
#
# Router base class
#
################################################################################

# use our own node class derived from the node class of network10.py
# so we can override routing behavior
class Router(Node):
    HELLO_INTERVAL = 5   # time between HELLO packets
    ADVERT_INTERVAL = 20  # time between route advertisements

    def __init__(self,location,address=None):
        Node.__init__(self, location, address=address)
        # additional instance variables
        self.neighbors = {}     # Link -> (timestamp, address, linkcost)
        self.routes = {}        # address -> Link
        self.routes[self.address] = 'Self'
        self.spcost = {}        # address -> shortest path cost to node
        self.spcost[self.address] = 0
        self.hello_offset = random.randint(0, self.HELLO_INTERVAL-1)
        self.ad_offset = random.randint(0, self.ADVERT_INTERVAL-1)
        self.hello_offset = 0
        self.ad_offset = 0

    def reset(self):
        Node.reset(self)
        self.spcost[self.address] = 0

    # return the link corresponding to a given neighbor, nbhr
    def getlink(self, nbhr):
        if self.address == nbhr: return None
        for l in self.links:
            if l.end2.address == nbhr or l.end1.address == nbhr:
                return l
	return None

    def peer(self, link):
        if link.end1.address == self.address: return link.end2.address
        if link.end2.address == self.address: return link.end1.address

    # use routing table to forward packet along appropriate outgoing link
    def forward(self,p):
        link = self.routes.get(p.destination, None)
        if link is None:
            print 'No route for ',p,' at node ',self
        else:
            print 'sending packet'
            link.send(self, p)

    def process(self,p,link,time):
        if p.type == 'HELLO':
            # remember addresses of our neighbors and time of latest update
            self.neighbors[link] = (time, p.source, link.cost)
        elif p.type == 'ADVERT':
            self.process_advertisement(p,link,time)
        elif p.type == 'DATA':
            self.process_data(p, time)
        else:
            Node.process(self, p, link, time)

    def process_advertisement(self,p,link,time):
        # will be filled in by the specific routing protocol
        return

    def process_data(self, p, time):
        return

    def sendHello(self, time):
        # STEP 1(a): send HELLO packets along all my links to neighbors
        # These periodic HELLOs tell our neighbors I'm still alive
        # The neighbors will get my address from the source address field
        for link in self.links:
            p = self.network.make_packet(self.address, self.peer(link),
                                         'HELLO', time,color='green')
            link.send(self,p)
        return

    def clearStaleHello(self, time):
        # STEP 1(b) : Look through neighbors table and eliminate
        # out-of-date entries.
        old = time - 2*self.HELLO_INTERVAL
        for link in self.neighbors.keys():
            if self.neighbors[link][0] <= old:
                del self.neighbors[link]
                self.link_failed(link)
        return

    def link_failed(self,link):
        pass

    def clear_routes(self,link):
        clear_list = []
        for dest in self.routes:
            if self.routes[dest] == link:
                clear_list.append(dest)
        for dest in clear_list:
            # print self.address, ' clearing route to ', dest
            del self.routes[dest]
            del self.spcost[dest]

    def transmit(self, time):
        if (time % self.HELLO_INTERVAL) == self.hello_offset:
            self.sendHello(time)
            self.clearStaleHello(time)
            self.send_pollution(time)
        if (time % self.ADVERT_INTERVAL) == self.ad_offset:
            self.send_advertisement(time)
        return

    def OnClick(self,which):
        if which == 'left':
            #print whatever debugging information you want to print
            print self
            print '  neighbors:',self.neighbors.values()
            print '  routes:'
            for (key,value) in self.routes.items():
                print '    ',key,': ',value, 'pathcost %.2f' % self.spcost[key]


# Network with link costs.  By default, the cost of a link is the
# Euclidean distance between the nodes at the ends of the link
class RouterNetwork(Network):
    def __init__(self,SIMTIME,NODES,LINKS,LOSSPROB):
        Network.__init__(self,SIMTIME)

        self.lossprob = LOSSPROB
        for n,r,c in NODES:
            self.add_node(r,c,address=n)
        for a1,a2 in LINKS:
            n1 = self.addresses[a1]
            n2 = self.addresses[a2]
            self.add_link(n1.location[0],n1.location[1],
                          n2.location[0],n2.location[1])

    # nodes should be an instance of LSNode (defined above)
    def make_node(self,loc,address=None):
        return Router(loc,address=address)

    def make_link(self,n1,n2):
        return LossyCostLink(n1,n2,self.lossprob)

#    def add_cost_link(self,x1,y1,x2,y2):
#        n1 = self.find_node(x1,y1)
#        n2 = self.find_node(x2,y2)
#        if n1 is not None and n2 is not None:
#            link = self.make_cost_link(n1,n2)
#            link.network = self
#            self.links.append(link)

    # reset network to its initial state
    def reset(self):
        # parent class handles the details
        Network.reset(self)
        # insert a single packet into the network with randomly
        # chosen source and destination.  Since we don't have code
        # to deliver the packet this just keeps the simulation alive...
        src = random.choice(self.nlist)
        dest = random.choice(self.nlist)
        src.add_packet(self.make_packet(src.location,dest.location,'DATA',1))


################################################################################
#
# Random graph generator
#
################################################################################

class RandomGraph:
    def __init__(self,numnodes=8):
        self.numnodes = numnodes
        if self.numnodes > 22:
            print "Maximum number of nodes = 22"
            self.numnodes = 26
        elif self.numnodes < 5:
            print "Minimum number of nodes = 5"
            self.numnodes = 5

        self.names = ['A', 'B', 'C', 'D', 'E',
                      'F', 'G', 'H', 'I', 'J',
                      'K', 'L', 'M', 'N', 'O',
                      'P', 'Q', 'R', 'S', 'T',
                      'U', 'V', 'W', 'X', 'Y', 'Z']
        self.position = [[10, 20],
                         [13.77722178, 15.96331309],
                         [5.60887984, 16.78558657],
                         [2.06452013, 8.95787052],
                         [19.77191006, 0.87228297],
                         [5.75550677, 2.60057144],
                         [5.38733916, 13.57671066],
                         [4.23256232, 5.31093319],
                         [9.83146319, 1.0672509],
                         [11.48235211, 2.9345715],
                         [11.78611074, 13.9951672],
                         [1.2, 3.28111976],
                         [11.88800315, 6.28358539],
                         [0.99906918, 10.71792812],
                         [13.2758929, 10.29778224],
                         [14.89189512, 5.23110081],
                         [18.06803831, 2.74949408],
                         [7.78552695, 10.14782577],
                         [0.35353674, 0.30708394],
                         [18.55017161, 6.95531719],
                         [17.5, 12.5],
                         [7.5, -2.5]]
        self.dist_matr = []

    def rss_report(self, ind, x, y):
        (nx, ny) = self.getCoord(ind)
        rss = log_path_model(sci_dist.euclidean([nx, ny], [x, y]))
        return rss

    def getCoord(self, i):
        x = self.position[i][0]
        y = self.position[i][1]
        return x, y

    def getIndex(self, x, y):
        if x<0 or y < 0 or x>=self.maxCols or y >= self.maxRows:
            return -1
        ind = y*self.maxCols + x
        if ind < self.numnodes:
            return ind
        else:
            return -1

    def getRange(self):
        self.dist_matr = sci_dist.squareform(sci_dist.pdist(self.position))
        min_ranges = []
        for row in self.dist_matr:
            tmp = sorted(row)
            min_ranges.append(tmp[1])
        eff_range = max(min_ranges) + 0.3
        return eff_range

    def rss2dist(self, rssi):
        # TODO: translate rssi into physical distance
        pass
        return dist

    def getAllNgbrs_distance(self, i):
        (x, y) = self.getCoord(i)
        ngbrs = []
        candiNgbrs = range(self.numnodes)
        candiNgbrs.pop(i)
        for nodeInd in candiNgbrs:
            rssi = self.rss_report(nodeInd, x, y)
            trans_dist = self.rss2dist(rssi) # rss2dist(rssi) to complete

            # TODO: vary distance threshold to see its impacts on tree structure (miniumum is 6)
            dist_threshold = 6

            if trans_dist <= dist_threshold:
                ngbrs.append(nodeInd)

        return ngbrs

    def checkLinkExists(self, links, a, b):
        for (c,d) in links:
            if a==c and b==d:
                return True
            if a==d and b==c:
                return True
        return False

    def genGraph(self):
        NODES = []
        LINKS = []

        for i in range(self.numnodes):
            (x,y) = self.getCoord(i)
            name = self.names[i]
            NODES.append((name,x,y))

        for i in range(self.numnodes):
            ngbrs = self.getAllNgbrs_distance(i)

            for n in ngbrs:
                if not self.checkLinkExists(LINKS, self.names[i], self.names[n]):
                    LINKS.append((self.names[i], self.names[n]))

        return (NODES, LINKS)

    def drawGraph(self):
        plt.close()
        fig, ax = plt.subplots()
        lines = []
        for i in range(self.numnodes):
            (x, y) = self.getCoord(i)
            if i == 0:
                plt.scatter(x, y, s=350, facecolors='none', marker='s', edgecolors='#EE9A00',linewidths=2)
                plt.text(x - 0.3, y - 0.38, self.names[i], fontsize=14)
            else:
                plt.scatter(x, y, s=280, facecolors='none', edgecolors='#551A8B',linewidths =1.5)
                plt.text(x - 0.27, y - 0.3, self.names[i], fontsize=12)
            ngbrs = self.getAllNgbrs_distance(i)
            # Creating line obj
            for ng in ngbrs:
                (nx, ny) = self.getCoord(ng)
                line = [(x, y), (nx, ny)]
                line_1 = [(nx, ny), (x, y)]
                if line_1 not in lines:
                    lines.append(line)
        lc = mc.LineCollection(lines, colors='#FF69B4', linewidths=2)
        ax.add_collection(lc)
        plt.show()
