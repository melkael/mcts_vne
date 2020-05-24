import pprint
from agent import Agent
from graph_classes import SN, VNR
from visualization import saveSNImage

import networkx as nx

from networkx.readwrite import json_graph
import json

def main():
    acc = 0
    sn = SN(n_nodes_min=15, n_nodes_max=15, beta=0.5, alpha=0.4)
    g = nx.Graph()
    g.add_node(0, cpu_max=10, cpu_used=0)
    g.add_node(1, cpu_max=10, cpu_used=0)
    g.add_node(2, cpu_max=10, cpu_used=0)
    g.add_node(3, cpu_max=10, cpu_used=0)
    g.add_node(4, cpu_max=10, cpu_used=0)
    g.add_node(5, cpu_max=10, cpu_used=0)

    g.add_edge(0, 1, BW_max=1, BW_used=0)
    g.add_edge(1, 2, BW_max=1, BW_used=0)
    g.add_edge(1, 3, BW_max=1, BW_used=0)
    g.add_edge(2, 3, BW_max=1, BW_used=0)
    g.add_edge(0, 4, BW_max=3, BW_used=0)
    g.add_edge(4, 5, BW_max=3, BW_used=0)
    g.add_edge(5, 3, BW_max=3, BW_used=0)

    sn.graph = g
    agent = Agent(sn)

    vnr = VNR()
    g2 = nx.Graph()
    g2.add_node(0, cpu=1)
    g2.add_node(1, cpu=1)
    g2.add_edge(0, 1, BW=2)

    vnr.graph = g2
    sn.graph.nodes[0]['cpu_used'] += 1
    sn.graph.nodes[3]['cpu_used'] += 1
    vnr.graph.nodes[0]['sn_node'] = 0
    vnr.graph.nodes[1]['sn_node'] = 3
    saveSNImage(agent.substrate_network, './samplerdv/1.png')
    status, vnr = agent.VLiM_MCF(vnr, False)
    saveSNImage(agent.substrate_network, './samplerdv/2.png')
    sn.graph = g
    agent=Agent(sn)
    status, vnr = agent.VLiM_MCF(vnr, True)
    print(status)
    saveSNImage(agent.substrate_network, './samplerdv/3.png')
    '''
    for i in range(20):
        vnr = VNR(n_nodes_min=3, n_nodes_max=4)
        #pp = pprint.PrettyPrinter(indent=4)
        result, vnr_returned = agent.VNoM(vnr)
        if result == 'success_VNoM':
            status, vnr_returned = agent.VLiM_MCF(vnr_returned, False)
            if status == 'success VLiM':
                acc += 1
                saveSNImage(agent.substrate_network, './samplerdv/graph{}.png'.format(acc))
                print(acc)
                pp = pprint.PrettyPrinter(indent=4)
                pp.pprint(vnr_returned.graph.edges(data=True))
    print("successful embeddings:", acc)
'''
'''
pp = pprint.PrettyPrinter(indent=4)
def main():
    acc = 0
    sn = SN(n_nodes_min=15, n_nodes_max=15, beta=0.5, alpha=0.4)
    agent = Agent(sn)
    vnr_returned = None
    data = json_graph.node_link_data(sn.graph)
    with open('./json/graph{}.json'.format(acc), 'w') as f:
        json.dump(data, f, indent=4)
    for i in range(20):
        vnr = VNR(n_nodes_min=3, n_nodes_max=4)
        #pp = pprint.PrettyPrinter(indent=4)
        result, vnr_returned = agent.VNoM(vnr)
        print(result)
        if result == 'success_VNoM':
            status, vnr_returned = agent.VLiM_MCF(vnr_returned, False)
            if status == 'success VLiM':
                acc += 1
                data = json_graph.node_link_data(sn.graph)
                with open('./json/graph{}.json'.format(acc), 'w') as f:
                    json.dump(data, f, indent=4)
                print(acc)
                #pp.pprint(vnr_returned.graph.edges(data=True))
                print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
                pp.pprint(list(agent.substrate_network.graph.nodes(data=True)))
    print("successful embeddings:", acc)

'''
if __name__ == "__main__": 
    main()
