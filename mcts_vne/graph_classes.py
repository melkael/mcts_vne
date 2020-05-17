from random import expovariate, randint, choice, sample, uniform, shuffle, seed
import matplotlib.pyplot as plt
import networkx as nx

class Waxman_Graph_Container(object):
    def __init__(self, n_nodes, beta, alpha, max_degree=float("inf"), domain=(0, 0, 25, 25)):
        self.n_nodes = n_nodes
        self.max_degree = max_degree
        self.domain = domain
        self.beta= beta
        self.alpha = alpha
        self.graph = self.make_graph()
    
    def make_graph(self):
        G = nx.waxman_graph(self.n_nodes, self.beta, self.alpha, domain=self.domain)
        #G = nx.cycle_graph(self.n_nodes)
        #G = nx.cycle_graph(self.n_nodes)
        G = self.connect_graph(G)
        G = self.reduce_degree(G)
        #On réduit le degré si besoin
        #G = nx.circular_ladder_graph(self.n_nodes, create_using=None)
        #G = nx.connected_watts_strogatz_graph(self.n_nodes, 2, 0.2, tries=100, seed=None)    
        return G
    
    def connect_graph(self, G):
        while not(nx.is_connected(G)):
            components = list(nx.connected_components(G))
            c1, c2 = sample(components, 2)
            u = choice(list(c1))
            v = choice(list(c2))
            G.add_edge(u, v)
        return G
    

    #Needs refactoring
    #The good approach is probably to ditch the built-in Waxman graph function for a custom one that already takes max degrees into account
    def reduce_degree(self, G):
        if self.max_degree == float("inf"):
            return G
        
        for n in list(G.nodes):
            edges = list(G.edges(n))
            shuffle(edges)
            for e in edges:
                if G.degree(n) <= self.max_degree:
                    break
                else:
                    G = self.remove_edge_if_no_disconnection(e, G)    
            
            # Maybe removing any adjacent edge disconnects the graph, then one need to handle this by disconnecting 
            # and then reconnecting the graph with other nodes
            while G.degree(n) > self.max_degree:
                edges = list(G.edges(n))
                shuffle(edges)
                e = edges.pop()
                G.remove_edge(e[0], e[1])
                #Here the graph is disconnected
                S = [G.subgraph(c).copy() for c in nx.connected_components(G)]
                #We removed 1 edge from a connected graph so there are only 2 connected components
                D0 = list(S[0].degree)
                D1 = list(S[1].degree)
                #Only keep non-saturated nodes
                D0 = [deg for deg in D0 if deg[1] < self.max_degree]
                D1 = [deg for deg in D1 if deg[1] < self.max_degree]

                # handle the case where all nodes in connected components are saturated.
                # If it's the case they all have a degree > 1 and then one can safely remove one random edge without disconnecting the component
                # This will then free a degree on two nodes for connecting the components
                if D0 == []:
                    edges = list(S[0].edges)
                    shuffle(edges)
                    G.remove_edge(edges[0][0], edges[0][1])

                if D1 == []:
                    edges = list(S[1].edges)
                    shuffle(edges)
                    G.remove_edge(edges[0][0], edges[0][1])
                
                # We then retry to connect the two components
                S = [G.subgraph(c).copy() for c in nx.connected_components(G)]
                D0 = list(S[0].degree)
                D1 = list(S[1].degree)
                #Remove nodes that are already saturated from the possible hosts of the new edge
                D0 = [deg for deg in D0 if deg[1] < self.max_degree]
                D1 = [deg for deg in D1 if deg[1] < self.max_degree]

                #Connect 2 of the remaining nodes randomly
                u = choice(D0)[0]
                v = choice(D1)[0]
                G.add_edge(u, v)
        return G
    
    def remove_edge_if_no_disconnection(self, e, graph):
        graph.remove_edge(e[0], e[1])
        if not(nx.is_connected(graph)):
            graph.add_edge(e[0], e[1])
        return graph
    
    def set_graph(self, graph):
        self.graph = graph
        
class VNR(Waxman_Graph_Container):
    def __init__(self, 
                 beta=0.2, 
                 alpha=0.5, 
                 dist_min=15, 
                 dist_max=25, 
                 n_nodes_min=3, 
                 n_nodes_max=10, 
                 max_degree=3,
                 cpu_need_min=2,
                 cpu_need_max=20,
                 BW_need_min=1,
                 BW_need_max=50,
                 domain=(0, 0, 25, 25)):

        self.substrate_edges_used = None
        
        self.cpu_need_min = cpu_need_min
        self.cpu_need_max = cpu_need_max
        self.BW_need_min = BW_need_min
        self.BW_need_max = BW_need_max
        
        super().__init__(randint(n_nodes_min, n_nodes_max), beta, alpha, max_degree)
        self.generate_cpu_needs()
        self.generate_bandwidth_needs()
    
    def generate_cpu_needs(self):
        for i in range(self.n_nodes):
            self.graph.nodes[i]['cpu'] = randint(self.cpu_need_min, self.cpu_need_max)
    
    def generate_bandwidth_needs(self):
        for e in self.graph.edges:
            self.graph[e[0]][e[1]]['BW'] = randint(self.BW_need_min, self.BW_need_max)

#SN and VNR classes could probably be merged bu I feel like it's clearer and more modular for future versions of the model in that way

class SN(Waxman_Graph_Container):
    def __init__(self, 
                 beta=0.2, 
                 alpha=0.5, 
                 n_nodes_min=25, 
                 n_nodes_max=50, 
                 max_degree=5,
                 cpu_capa_min=50,
                 cpu_capa_max=100,
                 BW_capa_min=50,
                 BW_capa_max=100,
                 domain=(0, 0, 25, 25)):
        
        self.cpu_capa_min = cpu_capa_min
        self.cpu_capa_max = cpu_capa_max
        self.BW_capa_min = BW_capa_min
        self.BW_capa_max = BW_capa_max
        
        super().__init__(randint(n_nodes_min, n_nodes_max), beta, alpha, max_degree)
        self.generate_cpu_capa()
        self.generate_bandwidth_max()
    
    def generate_cpu_capa(self):
        for i in range(self.n_nodes):
            self.graph.nodes[i]['cpu_max'] = randint(self.cpu_capa_min, self.cpu_capa_max)
            self.graph.nodes[i]['cpu_used'] = 0
    def generate_bandwidth_max(self):
        for e in self.graph.edges:
            self.graph[e[0]][e[1]]['BW_max'] = randint(self.BW_capa_min, self.BW_capa_max)
            self.graph[e[0]][e[1]]['BW_used'] = 0