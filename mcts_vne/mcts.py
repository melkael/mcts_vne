from graph_classes import SN, VNR
import copy
import networkx as nx
from math import sqrt
import numpy as np
import random
import sys
import pprint
import resource
import psutil
import os
import time

cpu_price = 2
BW_price  = 2
BW_cost   = 0.75


class MCTS(object):
    def __init__(self, sn, vnr):
        self.tree = nx.DiGraph()
        self.vnr = copy.deepcopy(vnr)
        self.name_generator = self.node_name_generator()

        #for n in self.sn.graph.nodes:
        #    self.sn.graph.nodes[n]['hosted_nfs'] = []

        # visits is the number of times this node was visited
        # value is the current value MCTS calculated for this node/state
        # next_nf is the index of the next nf to embed.
        # unexpanded children are the children of the nodes that have never been visited, these are implicit nodes, since their values
        # are always 0, their visits are always 0 and their next_nf is current(next_nf) + 1. Thus we don't need to store the values
        # On the other hand, expanded_children are nodes of the graph that we need to fully store since their visit and value counts can vary
        self.tree.add_node('root',
                           visits=0,
                           value=0,
                           nf_to_place=0,
                           unexpanded_children=self.get_legal_actions(0, sn),
                           used_sn=[],
                           sn=sn,
                           vnr=self.vnr
                           )
    
    def get_legal_actions(self, nf_index, sn):
        list_possible_actions = list(sn.graph.nodes(data=False))
        for n in sn.graph.nodes(data=True):
            
            # if the physical node is too far from the ideal location, then the action is illegal
            if self.euclidian_dist(n[1], self.vnr.graph.nodes[nf_index]) > self.vnr.global_dist_max:
                list_possible_actions.remove(n[0])

            # if the pysical node doesn't have enough free cpu capacity, then the action is illegal
            elif n[1]['cpu_max'] - n[1]['cpu_used'] < self.vnr.graph.nodes(data=True)[nf_index]['cpu']:
                list_possible_actions.remove(n[0])

        return list_possible_actions
    
    def euclidian_dist(self, n1, n2):

        x1 = n1['pos'][0]
        y1 = n1['pos'][1]
        x2 = n2['pos'][0]
        y2 = n2['pos'][1]

        return sqrt((x1 - x2)**2 + (y1 - y2) ** 2)

    def run(self, root):
        beta = 0
        while (beta < 40):
            beta += 1
            vl = self.treePolicy(root)
            delta = self.defaultPolicy(vl)
            self.backpropagate(delta, vl)
        
        max_avg_value = -1000000000000000000000
        # if there is no edge from root, then there are no available action : the network is too saturated to host this vnr
        if list(self.tree.edges(root, data=True)) == []:
            best_action = None
            new_root = None

        for k in list(self.tree.edges(root, data=True)):
            node_name = k[1]
            v = self.tree.nodes[node_name]['value']
            visits = self.tree.nodes[node_name]['visits']
            if max_avg_value < v/visits:
                best_action = k[-1]['action']
                new_root = k[1]
                max_avg_value = v/visits
        
        return best_action, new_root
    
    def backpropagate(self, delta, node_name):
        while node_name != 'root':
            self.tree.nodes[node_name]['value'] += delta
            self.tree.nodes[node_name]['visits'] += 1
            node_name = self.tree.nodes[node_name]['parent']
        # we also need to update root
        self.tree.nodes['root']['value'] += delta
        self.tree.nodes['root']['visits'] += 1

    def treePolicy(self, node_name):
        node = self.tree.nodes(data=True)[node_name]
        # while node has children, expanded or not, ie while it's non terminal
        while not (node['unexpanded_children'] == [] and list(self.tree.edges(node_name)) == []):
            # if there are unexpanded nodes, then expand one of them and return it
            if node['unexpanded_children'] != []:
                return self.expand(node_name)
            # go down in the tree choosing the best child
            else:
                node_name = self.BestChild(node_name)
                node = self.tree.nodes(data=True)[node_name]
        return node_name

    def defaultPolicy(self, node_name):
        # retrieving the state of the substrate node
        pp = pprint.PrettyPrinter(indent=4)
        sn = copy.deepcopy(self.tree.nodes[node_name]['sn'])
        vnr = copy.deepcopy(self.tree.nodes[node_name]['vnr'])
        # retrieving substrate nodes we can't use
        used_sn = copy.deepcopy(self.tree.nodes[node_name]['used_sn'])
        # we will place nfs starting from the one we have to place now
        nf_to_place = self.tree.nodes[node_name]['nf_to_place']

        # at most we place all the remaining nf
        while nf_to_place < len(self.vnr.graph.nodes):
            possible_actions = [x for x in self.get_legal_actions(nf_to_place, sn) if x not in used_sn]
            #pp.pprint(list(sn.graph.nodes(data=True)))
            #print(vnr.graph.nodes[nf_to_place])
            #print(possible_actions)

            # no more legal actions, embedding has failed
            if possible_actions == []: #and nf_to_place != len(self.vnr.graph.nodes) -1:
                return -100000000000
            else:
                # choose next host substrate node randomly
                sn_chosen = random.choice(possible_actions)
                # make sure the chosen sn node won't be reused
                used_sn.append(sn_chosen)
                # update cpu usage (normally we don't need to check for overflows since we filter in self.get_legal_actions)
                sn.graph.nodes[sn_chosen]['cpu_used'] += self.vnr.graph.nodes[nf_to_place]['cpu']
                vnr.graph.nodes[nf_to_place]['sn_node'] = sn_chosen
            
            # go to next nf
            nf_to_place += 1

        agent = Agent(sn)
        # if the node was terminal, then there was no simulation to make, just run vlim the first time we see the node
        if self.tree.nodes[node_name]['nf_to_place'] == len(self.vnr.graph.nodes):
            # if we've never seen it, then perform VLiM and store the result for future lookup
            if 'already seen' not in self.tree.nodes[node_name]:
                self.tree.nodes[node_name]['already seen'] = True
                result_VLiM, vnr = agent.VLiM_MCF(vnr)
                self.tree.nodes[node_name]['result_VLiM']  = result_VLiM
                self.tree.nodes[node_name]['result_VLiM_VNR'] = vnr
            result_VLiM, vnr = self.tree.nodes[node_name]['result_VLiM'], self.tree.nodes[node_name]['result_VLiM_VNR']
        # if the node is non-terminal then the result of the simulation could vary and thus we can't just store it once
        else:
            result_VLiM, vnr = agent.VLiM_MCF(vnr)
        
        if result_VLiM == 'success VLiM':
            reward = self.calculate_reward(vnr)
            #print(reward)
        else:
            reward = -100000000000
        return reward

    def calculate_reward(self, vnr):
        cpu = 0
        BW_used = 0
        BW_need = 0

        # we only need to track used cpu as used cpu is the same as virtual cpu
        for n in vnr.graph.nodes(data=True):
            cpu += n[-1]['cpu']
        
        for e in vnr.graph.edges(data=True):
            for _, BW in e[-1]['Virtual link']:
                BW_used += BW
        
        for e in vnr.graph.edges(data=True):
            BW_need += e[-1]['BW']

        global cpu_price
        global BW_price
        global BW_cost
        return cpu_price * cpu + BW_price * BW_need - BW_cost * BW_used
        

    def BestChild(self, node_name):
        children = [c[-1] for c in self.tree.edges(node_name)]
        def uct(c, D, parent_visits):
            node = self.tree.nodes(data=True)[c]
            return node['value'] / node['visits'] + D * sqrt(2*np.log(parent_visits)/node['visits']) 
        uct_list = []
        # Calculer tous les UCT pour en déduire le meilleur noeud
        for c in children:
            uct_list.append((c, uct(c, 2, self.tree.nodes[node_name]['visits'])))

        # find the best children and return it        
        uct_max = -100000000000000000000
        for (c, uct) in uct_list:
            if uct > uct_max:
                c_max = c
                uct_max = uct
        return c_max
        
    def expand(self, current_node):
        # the node chosen to be added to the tree corresponds to an action, eg a substrate node index (it's as if we chose
        # an arc of the tree, according, to the figure from the article)
        action_chosen = random.choice(self.tree.nodes[current_node]['unexpanded_children'])
        #remove the node from unexpanded childrens
        
        nf_to_embed = self.tree.nodes[current_node]['nf_to_place']
        cpu_required = self.vnr.graph.nodes[nf_to_embed]['cpu']

        # if there is enough cpu, place the nf on substrate node
        #if self.sn.graph.nodes[action_chosen]['cpu_max'] - self.sn.graph.nodes[action_chosen]['cpu_used'] > cpu_required:
        sn = copy.deepcopy(self.tree.nodes[current_node]['sn'])
        sn.graph.nodes[action_chosen]['cpu_used'] += cpu_required
        if sn.graph.nodes[action_chosen]['cpu_used'] > sn.graph.nodes[action_chosen]['cpu_max']:
            raise 'prout prout bug bug'

        vnr = copy.deepcopy(self.tree.nodes[current_node]['vnr'])
        vnr.graph.nodes[nf_to_embed]['sn_node'] = action_chosen
            
        name = self.name_generator.__next__()
            # the sn already used are the ones used before + the one we just chose
        used_sn = self.tree.nodes[current_node]['used_sn'] + [action_chosen]
            # if we reached the last nf, there are no possible actions anymore
        if nf_to_embed+1 < len(self.vnr.graph.nodes):
            possible_actions = [x for x in self.get_legal_actions(nf_to_embed+1, sn) if x not in used_sn]
        else:
            possible_actions = []

        self.tree.add_node(name,
                           visits=0,
                           value=0,
                           nf_to_place=nf_to_embed+1,
                           unexpanded_children=possible_actions,
                           used_sn=used_sn,
                           sn=sn,
                           parent=current_node,
                           vnr=vnr
                           )
        self.tree.add_edge(current_node, name, action=action_chosen)
        self.tree.nodes[current_node]['unexpanded_children'].remove(action_chosen)
            #print(name)
        return name
        #else:
            # si l'embedding fail ici alors le noeud n'entre pas dans l'arbre, normalement on n'entre jamais ici puisque les actions légales sont
            # filtrées en amont

            # la fonction get legal actions doit etre corrigée pour se baser sur chaque noeud et non sur le self.sn

        #    print(action_chosen)
        #    print(self.sn.graph.nodes[action_chosen]['cpu_max'] - self.sn.graph.nodes[action_chosen]['cpu_used'])
        #    print
        #    raise 'Erreur, action illégale'

    def node_name_generator(self):
        i = 0
        while True:
            i += 1
            yield i
    def has_children(self, node):
        return self.tree.nodes[node]['nf_to_place'] < len(self.tree.nodes['root']['vnr'].graph.nodes)

sn = SN(n_nodes_min=40, n_nodes_max=40)
pp = pprint.PrettyPrinter(indent=4)
begin = time.perf_counter()
for i in range(1):
    vnr = VNR(n_nodes_min=3, n_nodes_max=8)

    sn_backup = copy.deepcopy(sn)
    #vnr_backup = copy.deepcopy(vnr)

    mcts = MCTS(sn, vnr)
    root = 'root'
    a = time.perf_counter()
    k = 0
    print("VNR number", i)
    print("##############")
    while mcts.has_children(root):
        k += 1
        action, node = mcts.run(root)
        print("NF {} should be on SN node {}".format(k, action))
        root = node
        #print(action)
    if k == len(vnr.graph.nodes) and action != None and node != None:
        vnr = mcts.tree.nodes[node]['vnr']
        sn = mcts.tree.nodes[node]['sn']
        agent = Agent(sn)
        result_VLiM, vnr = agent.VLiM_MCF(vnr)
        if result_VLiM != 'success VLiM':
            status = 'VLiM failed'
            sn = sn_backup
        else:
            sn = agent.substrate_network
            status = 'success'
            print(vnr.graph.nodes(data=True))
            pp.pprint(list(vnr.graph.edges(data=True)))
            #print(vnr.graph.edges(data=True))
    else:
        status = 'VNoM failed'
        sn = sn_backup
    #if fail == 'success':
        #pp.pprint(list(vnr.graph.edges(data=True)))
        #pp.pprint(list(vnr.graph.nodes(data=True)))
        #pp.pprint(list(sn.graph.edges(data=True)))
        #pp.pprint(list(sn.graph.nodes(data=True)))
    print(status)
    b = time.perf_counter()
    print(b - a)
print(begin - time.perf_counter())
#pp.pprint(list(vnr.graph.nodes(data=True)))
