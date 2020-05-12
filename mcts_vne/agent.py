from graph_classes import SN, VNR
import random
import copy 
import networkx as nx

class Agent(object):
    def __init__(self, substrate_network):
        self.substrate_network = copy.deepcopy(substrate_network)

        self.vnr = None
        self.vnr_backup = None
        self.substrate_network_backup = None
    
    def initialize_embedding(self, vnr):
        self.vnr = copy.deepcopy(vnr)          # this is different from vnr return as at each step one removes a node from the current vnr
        self.vnr_returned = copy.deepcopy(vnr) # the vnr returned comprises the indexes of each substrate node for each NF
        self.vnr_backup = copy.deepcopy(vnr)
        self.substrate_network_backup = copy.deepcopy(self.substrate_network)

    def embed_current_nf(self, choose_substrate_node_method):

        if(len(self.vnr.graph.nodes) == 0):
            # embedding terminé
            return 'success_VNoM', self.vnr_returned
        current_nf_index = list(self.vnr.graph.nodes)[0]
        target_sn_node_index = choose_substrate_node_method(current_nf_index) # naive random for now, mcts later
        target_sn_node = self.substrate_network.graph.nodes(data=True)[target_sn_node_index]

        if target_sn_node['cpu_max'] - target_sn_node['cpu_used'] >= self.vnr.graph.nodes(data=True)[current_nf_index]['cpu']:
            target_sn_node['cpu_used'] += self.vnr.graph.nodes(data=True)[current_nf_index]['cpu']
            self.vnr_returned.graph.nodes[current_nf_index]["sn_node"] = target_sn_node_index
            self.vnr.graph.remove_node(current_nf_index)
            return 'success embedding', None

        else:
            self.substrate_network = self.substrate_network_backup
            return 'fail VNoM', None
        
    def VNoM(self, vnr):
        self.initialize_embedding(vnr)
        result_VNoM = 'success embedding'
        while result_VNoM == 'success embedding':
            result_VNoM, vnr_returned = self.embed_current_nf(self.choose_substrate_node_randomly)
        return result_VNoM, vnr_returned
        
    def VLiM(self, vnr):
        None

    def acknowledge_vnr_departure(self, vnr):
        # Currently edge removal is not implemented
        for n in vnr.graph.nodes(data=True):
            host = n[-1]['sn_node']
            self.substrate_network.graph.nodes[host]['cpu_used'] -= n[-1]['cpu']

    def choose_substrate_node_randomly(self, nf):
        return random.choice(list(self.substrate_network.graph.nodes))



def main():
    sn = SN()
    agent = Agent(sn)
    vnr_returned = None
    for i in range(1):
        vnr = VNR()
        result, vnr_returned = agent.VNoM(vnr)
        if result == 'success_VNoM':
            agent.acknowledge_vnr_departure(vnr_returned)

if __name__ == "__main__":
    main()