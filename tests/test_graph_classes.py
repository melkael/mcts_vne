import sys
import unittest
import networkx as nx
from random import seed, randint
sys.path.append('../')
from mcts_vne.graph_classes import Waxman_Graph_Container, SN, VNR


class WaxmanContainerTestCase(unittest.TestCase):
    def test_graph_is_connected(self):
        seed(a=0)
        for iteration in range(5):
            for n_nodes in range(2, 100):
                G = Waxman_Graph_Container(n_nodes, 0.2, 0.5)
                self.assertTrue(nx.is_connected(G.graph))
    
    def test_graph_has_n_nodes(self):
        for iteration in range(5):
            for n_nodes in range(2, 100):
                G = Waxman_Graph_Container(n_nodes, 0.2, 0.5)
                self.assertEqual(G.graph.number_of_nodes(), n_nodes)
    
    def test_graph_respects_max_degree(self):
        for n_nodes in range(2, 100):
            for max_degree_allowed in range(2, 10):
                G = Waxman_Graph_Container(n_nodes, 0.2, 0.5, max_degree_allowed)
                degrees = sorted(G.graph.degree, key=lambda x: x[1], reverse=True)
                max_degree = degrees[0][1]
                self.assertLessEqual(max_degree, max_degree_allowed)

    def test_graph_is_connected_with_max_degrees(self):
        for iteration in range(5):
            for max_degree_allowed in range(2, 10):
                for n_nodes in range(2, 100):
                    G = Waxman_Graph_Container(n_nodes, 0.2, 0.5, max_degree_allowed)
                    self.assertTrue(nx.is_connected(G.graph))
    
    def test_graph_has_n_nodes_with_max_degrees(self):
        for iteration in range(5):
            for max_degree_allowed in range(2, 10):
                for n_nodes in range(2, 100):
                    G = Waxman_Graph_Container(n_nodes, 0.2, 0.5, max_degree_allowed)
                    self.assertEqual(G.graph.number_of_nodes(), n_nodes)

class SNTestCase(unittest.TestCase):
    def test_cpu_max_is_in_bounds(self):
        max_cpu = 1000
        min_cpu = 1
        for i in range(100):
            sn = SN(0.2, 0.5, n_nodes_min=2, n_nodes_max=100, max_degree=randint(2, 20), cpu_capa_min=min_cpu, cpu_capa_max=max_cpu)
            for i in sn.graph.nodes(data=True):
                self.assertLessEqual(i[-1]['cpu_max'], max_cpu)
                self.assertLessEqual(min_cpu, i[-1]['cpu_max'])
    
    def test_cpu_used_is_0(self):
        max_BW = 1000
        min_BW = 1
        for i in range(100):
            sn = SN(0.2, 0.5, n_nodes_min=2, n_nodes_max=100, max_degree=randint(2, 20), cpu_capa_min=1, cpu_capa_max=1000, BW_capa_min=min_BW, BW_capa_max=max_BW)
            for i in sn.graph.nodes(data=True):
                self.assertEqual(i[-1]['cpu_used'], 0)

    def test_BW_is_in_bounds(self):
        max_BW = 1000
        min_BW = 1
        for i in range(100):
            sn = SN(0.2, 0.5, n_nodes_min=2, n_nodes_max=100, max_degree=randint(2, 20), cpu_capa_min=1, cpu_capa_max=1000, BW_capa_min=min_BW, BW_capa_max=max_BW)
            for i in sn.graph.edges(data=True):
                self.assertLessEqual(i[-1]['BW_max'], max_BW)
                self.assertLessEqual(min_BW, i[-1]['BW_max'])
    
    def test_BW_used_is_0(self):
        max_BW = 1000
        min_BW = 1
        for i in range(100):
            sn = SN(0.2, 0.5, n_nodes_min=2, n_nodes_max=100, max_degree=randint(2, 20), cpu_capa_min=1, cpu_capa_max=1000, BW_capa_min=min_BW, BW_capa_max=max_BW)
            for i in sn.graph.edges(data=True):
                self.assertEqual(i[-1]['BW_used'], 0)

class VNRTestCase(unittest.TestCase):
    def test_cpu_is_in_bounds(self):
        max_cpu = 1000
        min_cpu = 1
        for i in range(100):
            vnr = VNR(0.2, 0.5, n_nodes_min=2, n_nodes_max=100, max_degree=randint(2, 20), cpu_need_min=min_cpu, cpu_need_max=max_cpu)
            for i in vnr.graph.nodes(data=True):
                self.assertLessEqual(i[-1]['cpu'], max_cpu)
                self.assertLessEqual(min_cpu, i[-1]['cpu'])
    
    def test_BW_is_in_bounds(self):
        max_BW = 1000
        min_BW = 1
        for i in range(100):
            vnr = VNR(0.2, 0.5, n_nodes_min=2, n_nodes_max=100, max_degree=randint(2, 20), BW_need_min=min_BW, BW_need_max=max_BW)
            for i in vnr.graph.edges(data=True):
                self.assertLessEqual(i[-1]['BW'], max_BW)
                self.assertLessEqual(min_BW, i[-1]['BW'])

if __name__ == '__main__':
    unittest.main()