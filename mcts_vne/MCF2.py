from mip import *
import networkx as nx
import copy
from graph_classes import VNR, SN
import random
import time
import pprint
#from agent import Agent
import math

#random.seed(0)

# This class aims at providing an interface for solving the multicommodity flow problem for the VNE problem.
# Based on gurobi's netflow documentation example with tweaks and adaptations to make it work seamlessly with my networkx graph class.

# See Network Flows from Ahuja et al, chapter 17 for theoretical details

class MCF_vne(object):
    def __init__(self, vnr, sn):
        self.sn = sn
        self.vnr = vnr
        # keys of this map will be commodity names, values will be the associated edge in the vnr
        self.map_commodities_vnr_edges = {}
        # keys here are the commodities, and values are the substrate node where the vnr nodes are hosted
        self.map_commodities_sn_nodes  = {}
        self.commodities = []

    # performs the transformation described  in Network Flows from Ahuja, Magnanti and Orlin, at page 689 (exercise 17.21)
    # eg (i - j) becomes (i -> i'), (j -> i'), (j' -> i), (j' -> j), (i' -> j') where all edges have a cost of 0 and BW of infinity
    # excepted for (i' -> j') that keeps the cost (in our case, always 1) and capacity of the undirected edge
    def undirected_to_directed_normalized(self, g):
        g = g.to_directed()

        map_directed_edge_undirected = {}

        #now the graph has every edge in both directions, we want to remove one of them
        for e in g.edges:
            if g.has_edge(e[1], e[0]):
                g.remove_edge(e[1], e[0])
        
        #perform the transformation for each edge
        edges = copy.deepcopy(g.edges(data=True))
        labels_generator = self.generator_unused_node_label()
        
        for e in edges:
            i = e[0]
            j = e[1]
            edge_datas = e[2]

            # generate names for new nodes
            i_prime = labels_generator.__next__()
            j_prime = labels_generator.__next__()
            
            g.add_node(i_prime)
            g.add_node(j_prime)

            g.remove_edge(i, j)
            
            g.add_edge(i, i_prime, BW_max=math.inf, BW_used=0)
            g.add_edge(j, i_prime, BW_max=math.inf, BW_used=0)
            g.add_edge(j_prime, i, BW_max=math.inf, BW_used=0)
            g.add_edge(j_prime, j, BW_max=math.inf, BW_used=0)

            # corresponding_edge is an attribute that enables us to track which oriented (newi, newj) arc corresponds to
            # a given edge (since the rest of the arcs corresponding to the oriented arc will have cost of 0 and capacity of infinity)
            g.add_edge(i_prime, j_prime, BW_max=edge_datas['BW_max'], BW_used=edge_datas['BW_used'], corresponding_edge=(i, j))
            map_directed_edge_undirected[(i_prime, j_prime)] = (e[0], e[1])
        return g, map_directed_edge_undirected

    # generates names for the new nodes introduced by the transformation.
    # names are new0, new1, etc
    def generator_unused_node_label(self):
        i = 0
        while True:
            yield 'new'+str(i)
            i += 1

    def multidict(self, dictionary):
        keys = []
        values = []
        for k in dictionary.keys():
            keys.append(k)
            values.append(dictionary[k])
        return keys, dictionary

    # generates a list of arcs and a list of capacities
    def get_arcs_capacity(self, sn_directed_graph):
        arcs_cap_dict = {}
        edges = list(sn_directed_graph.edges)
        sn_directed_graph.edges[edges[0]]
        for e in edges:
            arcs_cap_dict[e] = sn_directed_graph.edges[e]['BW_max'] - sn_directed_graph.edges[e]['BW_used']
        arcs, capacity = self.multidict(arcs_cap_dict)
        return arcs, capacity

    # generates the dict of costs for each arc. Every arc has a cost of 1, excepted the arcs added by the transformation
    # which have a cost of 0 (and a capacity of infinity, that was set earlier)
    def make_costs(self, arcs, commodities, capacity):
        cost = {}
        for arc in arcs:
            for c in commodities:
                if capacity[arc] == math.inf:
                    cost[str(c), arc] = 0
                else:
                    cost[str(c), arc] = 1
        return cost

    
    # This defines each flow's (commodity) requirements.
    # Each commodity represents an edge of capacity BW on the undirected arc
    # One of the corresponding nodes need to send BW and the other one recieves BW (eg sends -BW) 
    def make_inflow(self, arcs, nodes, commodities):
        inflow = {}
        # first set all inflows to 0
        for n in nodes:
            for c in commodities:
                inflow[(c, n)] = 0

        # then set the interesting nodes
        for c, (sn_n1, sn_n2) in self.map_commodities_sn_nodes.items():
            # the value of the flow between those 2 sn nodes is contained in the VNR graph, on the edge
            vnr_n1 = self.map_commodities_vnr_edges[c][0]
            vnr_n2 = self.map_commodities_vnr_edges[c][1]
            flow = self.vnr.graph.get_edge_data(vnr_n1, vnr_n2)["BW"]

            inflow[(c, sn_n1)] = -flow
            inflow[(c, sn_n2)] = flow
        return inflow

    # each commodity (ie virtual link) needs it's own name
    def generator_unused_commodity_label(self):
        i = 0
        while True:
            yield 'c'+str(i)
            i += 1

    # commodities are built from the virtual edges, each virtual edge gives a source and a destination
    # (the 2 physical nodes that host the 2 NFs of that virtual edge) for a flow of BW
    def make_commodities(self):
        labels_generator = self.generator_unused_commodity_label()
        #print(self.vnr)
        for e in self.vnr.graph.edges:
            label =  labels_generator.__next__()
            self.commodities.append(label)
            self.map_commodities_vnr_edges[label] = e
            self.map_commodities_sn_nodes[label]  = (self.vnr.graph.nodes[e[0]]['sn_node'],
                                                     self.vnr.graph.nodes[e[1]]['sn_node'])
    
    def solve(self, stressConstraint=False):
        sn_directed_graph, map_directed_edge_undirected = self.undirected_to_directed_normalized(self.sn.graph)
        self.make_commodities()
        commodities = self.commodities
        nodes = list(sn_directed_graph.nodes)
        arcs, capacity = self.get_arcs_capacity(sn_directed_graph)
        cost = self.make_costs(arcs, commodities, capacity)
        inflow = self.make_inflow(arcs, nodes, commodities)

        # build the model
        m = Model(sense=MINIMIZE, solver_name=CBC)
        m.verbose = 0
        # create variables
        #flow = m.addVars(commodities, arcs, obj=cost, name="flow")

        flow = {}

        for c in commodities:
            flow[c] = {}
            for i, j in arcs:
                flow[c][i, j] = m.add_var()
        

        #Arc capacity constraint : an arc cannot exceed it's capacity
        for i, j in arcs:
            m.add_constr(xsum(flow[c][i, j] for c in commodities) <= capacity[i, j])

        # Flow conservation constraints

        # a changer, ajouter aussi les inflows
        for n in nodes:
            out_ = []
            in_  = []
            for i, j in arcs:
                if i == n:
                    out_.append((i, j))
                elif j == n:
                    in_.append((i, j))

            for c in commodities:
                out_links = list(flow[c][i, j] for (i, j) in out_)
                out_links.append(inflow[c, n])
                in_links  = list(flow[c][i, j] for (i, j) in in_ )
                m.add_constr(xsum(out_links) == xsum(in_links))
        

        
        obj = []
        for c in commodities:
            for (i,j) in arcs:
                obj.append(cost[c, (i,j)] * flow[c][i, j])
        
        m.objective = minimize(xsum(obj))
        
        status = m.optimize()
        '''print('optimal solution cost {} found'.format(m.objective_value))
        if status == OptimizationStatus.OPTIMAL or status == OptimizationStatus.FEASIBLE:
            print('solution:')
            for v in m.vars:
                if abs(v.x) > 1e-6: # only printing non-zeros
                    print('{} : {}'.format(v.name, v.x))
        '''
        flow_returned = {}

        if status != OptimizationStatus.INFEASIBLE:
            for c in commodities:
                flow_returned[self.map_commodities_vnr_edges[c]] = {}
                for i, j in arcs:
                    if abs(flow[c][i, j].x) > 0 and (i, j) in map_directed_edge_undirected:
                        n1 = map_directed_edge_undirected[(i,j)][0]
                        n2 = map_directed_edge_undirected[(i,j)][1]

                        flow_returned[self.map_commodities_vnr_edges[c]][(n1, n2)] = flow[c][i, j].x
        return status, flow_returned
        '''
        m.addConstrs(
            (gp.quicksum(flow[h, i, j] for i, j in arcs.select('*', j)) + inflow[h, j] ==
            gp.quicksum(flow[h, j, k] for j, k in arcs.select(j, '*'))
            for h in commodities for j in nodes), "node")
        '''
        #for i, j in arcs:
        #    m.add_constr(xsum(flow[h, i, j] for h in commodities) <= capacity[i, j])
        #for i, j in arcs:
        #    m.addConstr(sum(flow[h, i, j] for h in commodities) <= capacity[i, j],
        #                "cap[%s, %s]" % (i, j))

        #m.add_constr(m.x)

        '''
        #Arc capacity constraint : an arc cannot exceed it's capacity
        for i, j in arcs:
            m.addConstr(sum(flow[h, i, j] for h in commodities) <= capacity[i, j],
                        "cap[%s, %s]" % (i, j))

        # Flow conservation constraints
        m.addConstrs(
            (gp.quicksum(flow[h, i, j] for i, j in arcs.select('*', j)) + inflow[h, j] ==
            gp.quicksum(flow[h, j, k] for j, k in arcs.select(j, '*'))
            for h in commodities for j in nodes), "node")
        
        # define objective function
        obj = gp.LinExpr()
        coeffs = []
        vars = []

        for c in commodities:
            for i, j in arcs:
                if (c, (i, j)) in cost:
                    coeffs.append(cost[c, (i, j)])
                    vars.append(flow[c, i, j])
        obj.addTerms(coeffs, vars)

        if stressConstraint == True:
            max_link_usage = m.addVar(name='max_link_usage')
            
            for i, j in arcs:
                m.addConstr((1/(capacity[i, j]+0.00000000000001)) * sum(flow[h, i, j] for h in commodities) <= max_link_usage,
                            "cap[%s, %s]" % (i, j))
            obj.addTerms([len(nodes)], [max_link_usage])

        m.setObjective(obj, GRB.MINIMIZE)

        # Compute optimal solution
        m.optimize()

        # Build solution dict
        flow_returned = {}
        if m.status == GRB.OPTIMAL:
            solution = m.getAttr('x', flow)
            for h in commodities:
                flow_returned[self.map_commodities_vnr_edges[h]] = {}
                for i, j in arcs:
                    if solution[h, i, j] > 0 and (i, j) in map_directed_edge_undirected:
                        n1 = map_directed_edge_undirected[(i,j)][0]
                        n2 = map_directed_edge_undirected[(i,j)][1]

                        flow_returned[self.map_commodities_vnr_edges[h]][(n1, n2)] = solution[h, i, j] 
        # m.status tells us if the embedding was feasible or not
        # flow_returned is the solution into a dict of dicts  of the following form : 
        # flow_returned[virtual link] : [physical link] : flow value 
        return m.status, flow_returned


# if you want to try the methods directly here, from Agent import agent. Comment the mcf import in agent.py to avoid infinite imports 
'''
'''
random.seed(0)
sn = SN(n_nodes_min=10,n_nodes_max=10)
agent = Agent(sn)
a = time.perf_counter()

for i in range(100):
    
    vnr_returned = None
    vnr = VNR(n_nodes_min=2, n_nodes_max=2)
    
    result, vnr_returned = agent.VNoM(vnr)
    if result != 'fail VNoM':
        mcf = MCF_vne(vnr_returned, sn)
        status, flow_returned = mcf.solve()
        #print(status)

print(a - time.perf_counter())
'''