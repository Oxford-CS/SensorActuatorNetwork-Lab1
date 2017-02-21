### Tree routing protocol
import numpy as np
from matplotlib.lines import Line2D

from dependency.set_up import *

MEASUREMENT_INTERVAL = 5
INTERVAL = 600


def max_value_list(li, interval, time):
    max_val = 0
    output = None
    for item in li:
        if max_val < item[0] and item[1] >= time - interval:
            max_val = item[0]
            output = item
    return output


def max_value_dict_2(dict):
    max_val = 0
    max_ad = None
    for item in dict.iteritems():
        ad = item[0]
        val = item[1][0]
        if val >= max_val:
            # max and not expiry
            max_val = val
            max_ad = ad
    return max_ad


class TreeRouter(Router):
    def __init__(self, location, address=None):
        Router.__init__(self, location, address=address)
        if self.address == 'A' or self.address == 'V':
            self.hopCount = 0 # Sink node with hop count 0
            self.pollution = {}
        else:
            self.hopCount = sys.maxsize
            self.pollution = {self.address: (0, 0)}
        self.parent = None
        self.trs_time = None

        self.measurements = []
        self.local_max = None

    def send_pollution(self, time):
        if self.address != 'A':
            data = self.make_data(time)
            if data is not None:
                for link in self.links:
                        if link.end1.address == self.parent or link.end2.address == self.parent:
                            # print('pollution of node {} is {}'.format(self.address, data))
                            p = self.network.make_packet(self.address, self.peer(link),
                                                         'DATA', time,
                                                         color='red', ad=data)
                            link.send(self, p)

    def send_advertisement(self, time):
        adv = self.make_tree_advertisement()
        for link in self.links:
            p = self.network.make_packet(self.address, self.peer(link),
                                         'ADVERT', time,
                                         color='red', ad=adv)
            link.send(self, p)

    def make_data(self, time):
        measurement = np.random.randint(1000)
        self.measurements.append((measurement, time))
        self.local_max = max_value_list(self.measurements, INTERVAL, time)
        # print('At time {}, node {} has local max of {}'.format(time, self.address, self.local_max))
        # if self.local_max[0] > self.pollution[self.address][0]:
        self.pollution[self.address] = self.local_max
        # find the key with largest value
        # ad = max_value_dict(self.pollution, time, INTERVAL)
        ad = max_value_dict_2(self.pollution)
        if ad is None:
            return ad
        else:
            return ad, self.pollution[ad]

    def dic_update(self, time, interval):
        bad_ass = []
        for item in self.pollution.iteritems():
            ad = item[0]
            timestemp = item[1][1]
            # if val >= max_val:
            if timestemp < time - interval - MEASUREMENT_INTERVAL and ad is not self.address:
                bad_ass.append(ad)
        for ad in bad_ass:
            # print('deleting {} in node {}'.format(self.pollution[ad], self.address))
            del self.pollution[ad]

            # Make a distance vector protocol advertisement, which will be sent
    # by the caller along all the links
    def make_tree_advertisement(self):
        # nodeHopCount = beaconHopCount + 1
        return (self.address, self.hopCount)

    def link_failed(self, link):
        # If a link is broken, remove it from my routing/cost table
        self.clear_routes(self)

    def process_advertisement(self, p, link, time):
        self.integrate(p.properties['ad'], p.start, time)

    def process_data(self, p, time):
        data = p.properties['ad']
        self.pollution[data[0]]= data[1] # update the pollution dictionary
        self.dic_update(time, INTERVAL)

    # Integrate new routing advertisement to update routing
    # table and costs
    def integrate(self, adv, t_send, t_rece):
        # Update purse hopCount and associate it with its parent
        dst = adv[0]
        dst_cost = adv[1]
        if dst_cost + 1 < self.hopCount:
            self.parent = dst
            self.hopCount = dst_cost + 1
            self.trs_time = t_rece - t_send
        elif dst_cost+1 == self.hopCount and (t_rece - t_send) < self.trs_time:
            self.parent = dst # same hop number, but less transmission time, change parent only
            self.trs_time = t_rece - t_send


# A network with nodes of type DVRouter.
class TreeRouterNetwork(RouterNetwork):
    # nodes should be an instance of DVNode (defined above)
    def make_node(self,loc,address=None):
        return TreeRouter(loc,address=address)


def make_proxy(color, **kwargs):
    return Line2D([0, 1], [0, 1], color=color, **kwargs)


def show_tree(net):
    plt.close()
    fig, ax = plt.subplots()
    color_sink = np.array([x for x in 'bgrcymkbgrcmykbgrcmykbgrcmyk'])
    lines = []
    colors = []
    max_hop = 0
    for i in range(net.numnodes):
        node = net.nlist[i]
        (x, y) = node.location
        if i == 0 or i == 21:
            plt.scatter(x, y, s=350, facecolors='none', marker='s',
                        edgecolors='#EE9A00', linewidths=2)
            plt.text(x - 0.3, y - 0.38, node.address, fontsize=14)
        else:
            plt.scatter(x, y, s=280, facecolors='none', edgecolors='#551A8B', linewidths=1.5)
            plt.text(x - 0.27, y - 0.3, node.address, fontsize=12)
        if node.parent is None:
            continue
        parent_address = node.parent
        parent = net.addresses[parent_address]
        (px, py) = parent.location
        # Creating line obj bwtween parent and children
        line = [(x, y), (px, py)]
        lines.append(line)
        colors.append(color_sink[node.hopCount-1])
        if node.hopCount > max_hop:
            max_hop = node.hopCount

    lc = mc.LineCollection(lines, colors=colors, linewidths=2)
    ax.add_collection(lc)
    z = color_sink[:max_hop]
    proxies = [make_proxy(item, linewidth=5) for item in z]
    ax.legend(proxies, ['HopCount = 1', 'HopCount = 2', 'HopCount = 3', 'HopCount = 4',
                        'HopCount = 5', 'HopCount = 6'])
    plt.show()


########################################################################

if __name__ == '__main__':
    graph = RandomGraph(21)
    NODES, LINKS = graph.genGraph()
    net = TreeRouterNetwork(4000, NODES, LINKS, 0)
    net.set_nodes(len(net.addresses))
    net.reset()
    net.step(count = 2000)
    show_tree(net)
    for i in range(net.numnodes):
        node = net.nlist[i]
    na = net.nlist[0]
    ad = max_value_dict_2(na.pollution)
