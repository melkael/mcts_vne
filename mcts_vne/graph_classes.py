from random import expovariate, randint, choice, sample, uniform, shuffle
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
        G = self.connect_graph(G)
        G = self.reduce_degree(G)
        #On réduit le degré si besoin
        
        return G
    
    def connect_graph(self, G):
        while not(nx.is_connected(G)):
            components = list(nx.connected_components(G))
            c1, c2 = sample(components, 2)
            u = choice(list(c1))
            v = choice(list(c2))
            G.add_edge(u, v)
        return G
    
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
        # here the degrees are mostly below max_degree excepted a few ones.
        # One needs to remove the extra edges (which will disconnect the graph) and then connect the newly-created
        # Connected components between each other without exceeding max_degree
        return G
    
    def remove_edge_if_no_disconnection(self, e, graph):
        graph.remove_edge(e[0], e[1])
        if not(nx.is_connected(graph)):
            graph.add_edge(e[0], e[1])
        return graph


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
                 BW_need_min=0,
                 BW_need_max=50,
                 domain=(0, 0, 25, 25)):
        
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
                 BW_capa_min=0,
                 BW_capa_max=50,
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