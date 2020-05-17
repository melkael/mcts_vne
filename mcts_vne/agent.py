from graph_classes import SN, VNR
import random
import copy 
import pprint
import networkx as nx
from MCF import MCF_vne

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

        self.already_used_nodes = []

    # choose_substrate_node_method is the method that will be called in order to embed the current nf (ie VNR node)
    def embed_current_nf(self, choose_substrate_node_method):
        if(len(self.vnr.graph.nodes) == 0):
            # embedding terminé
            return 'success_VNoM', self.vnr_returned
        # choose the next vnr node (nf)
        current_nf_index = list(self.vnr.graph.nodes)[0]
        # then choose it's host node using the selected method
        target_sn_node_index = choose_substrate_node_method(current_nf_index) # naive random for now, mcts later
        
        # TODO (the next code line and the if statement deserve to be in a separate function)
        # we have the substrate node index, retrieve the associated data (ie node characteristics )
        target_sn_node = self.substrate_network.graph.nodes(data=True)[target_sn_node_index]
        # then if the virtual node is hostable host it
        if target_sn_node['cpu_max'] - target_sn_node['cpu_used'] >= self.vnr.graph.nodes(data=True)[current_nf_index]['cpu']:
            self.already_used_nodes.append(target_sn_node_index)
            target_sn_node['cpu_used'] += self.vnr.graph.nodes(data=True)[current_nf_index]['cpu']
            self.vnr_returned.graph.nodes[current_nf_index]["sn_node"] = target_sn_node_index
            self.vnr.graph.remove_node(current_nf_index)
            return 'success embedding', None

        # if not hostable then VNoM has failed
        else:
            self.substrate_network = self.substrate_network_backup
            return 'fail VNoM', None
        
    def VNoM(self, vnr):
        self.initialize_embedding(vnr)
        result_VNoM = 'success embedding'
        while result_VNoM == 'success embedding':
            result_VNoM, vnr_returned = self.embed_current_nf(self.choose_substrate_node_randomly)
        return result_VNoM, vnr_returned
        

    # TODO : clean that up and put it in a separate class
    def VLiM_BFS(self, vnr):
        list_substrate_edges_used = []
        for e_virt in vnr.graph.edges(data=True):            
            # compute shortest paths between host substrate nodes for each virtual edge
            host0 = vnr.graph.nodes(data=True)[e_virt[0]]['sn_node']
            host1 = vnr.graph.nodes(data=True)[e_virt[1]]['sn_node']
            BW_required = e_virt[2]['BW']

            shortest_paths = list(nx.all_shortest_paths(self.substrate_network.graph, source=host0, target=host1))
            shortest_path = random.choice(shortest_paths)

            # edge_list_sp is the list of edges that constitute the shortest path
            edge_list_sp = [(shortest_path[i], shortest_path[i+1]) for i in range(len(shortest_path)-1)]
            for e_phys in edge_list_sp:
                source = e_phys[0]
                target = e_phys[1]
                attributes = self.substrate_network.graph[source][target]
                if BW_required + attributes['BW_used'] < attributes['BW_max']:
                    print('ok')
                    attributes['BW_used'] += BW_required
                    list_substrate_edges_used.append([source, target, BW_required])
                else:
                    self.substrate_network = self.substrate_network_backup
                    return 'fail VLiM', None
        vnr.substrate_edges_used = list_substrate_edges_used
        return 'success VLiM', vnr
    
    # set stress constraint to True to get the result when trying to minimize stress on the edges
    # otherwise set it to False to get the minimum cost flow for the given VNR 
    def VLiM_MCF(self, vnr, stressConstraint=False):
        mcf = MCF_vne(vnr, self.substrate_network)
        return mcf.solve(stressConstraint=stressConstraint)

    # TODO next
    def acknowledge_vnr_departure(self, vnr):
        # Currently edge removal is not implemented
        for n in vnr.graph.nodes(data=True):
            host = n[-1]['sn_node']
            self.substrate_network.graph.nodes[host]['cpu_used'] -= n[-1]['cpu']

    # TODO : implement MCTS instead
    def choose_substrate_node_randomly(self, nf):
        choice = random.choice(list(self.substrate_network.graph.nodes))
        while choice in self.already_used_nodes:
            choice = random.choice(list(self.substrate_network.graph.nodes))
        return choice

def main():
    sn = SN()
    agent = Agent(sn)
    vnr_returned = None
    for i in range(1):
        vnr = VNR()
        pp = pprint.PrettyPrinter(indent=4)
        result, vnr_returned = agent.VNoM(vnr)
        status, dict_solution = agent.VLiM_MCF(vnr_returned, True)
        print(status)
        pp.pprint(dict_solution)
#        result2, vnr_returned = agent.VLiM_BFS(vnr_returned)
        #print(agent.substrate_network.graph.edges(data=True))
        #print
        #if result == 'success_VNoM':
        #    agent.acknowledge_vnr_departure(vnr_returned)

if __name__ == "__main__": 
    main()