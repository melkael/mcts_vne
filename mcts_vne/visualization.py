from graph_classes import VNR, SN
from pygraphviz import *
import networkx as nx
import copy 
#import matplotlib.pyplot as plt


def saveSNImage(sn, img):
    # we will modify the graph, make sure it's on a different instance
    sn = copy.deepcopy(sn)
    
    # graphviz bugs with euclidian positions and w/e we want to see the graph as clearly as possible
    #for node in sn.graph:
    #    del sn.graph.nodes[node]['pos']

    for u,v,d in sn.graph.edges(data=True):
        # set edge labels for graphviz
        l = str(d.get('BW_used','')) + '/' + str(d.get('BW_max',''))
        d['label'] = l
        # choose color according to usage
        percentage_use = 100 * d.get('BW_used','') / d.get('BW_max','') 
        if percentage_use > 80:
            d['color'] = 'red'
        elif percentage_use < 50:
            d['color'] = 'green'
        else:
            d['color'] = 'orange'
        

    for x,d in sn.graph.nodes(data=True):
        l = str(d.get('cpu_used','')) + '/' + str(d.get('cpu_max',''))
        d['label'] = str(x) + ':' + l

        percentage_use = 100 * d.get('cpu_used','') / d.get('cpu_max','')
        if percentage_use > 80:
            d['color'] = 'red'
        elif percentage_use < 50:
            d['color'] = 'green'
        else:
            d['color'] = 'orange'
    g  = nx.nx_agraph.to_agraph(sn.graph)
    g.graph_attr.update(K = 5)
    g.layout('dot')
    g.draw(img)