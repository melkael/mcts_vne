import sys
import unittest
import networkx as nx
from random import seed
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
            for max_degree_allowed in range(2, 7):
                G = Waxman_Graph_Container(n_nodes, 0.2, 0.5, max_degree_allowed)
                degrees = sorted(G.graph.degree, key=lambda x: x[1], reverse=True)
                max_degree = degrees[0][1]
                self.assertLessEqual(max_degree, max_degree_allowed)

    def test_graph_is_connected_with_max_degrees(self):
        for iteration in range(5):
            for max_degree_allowed in range(2, 7):
                for n_nodes in range(2, 100):
                    G = Waxman_Graph_Container(n_nodes, 0.2, 0.5, max_degree_allowed)
                    self.assertTrue(nx.is_connected(G.graph))
    
    def test_graph_has_n_nodes_with_max_degrees(self):
        for iteration in range(5):
            for max_degree_allowed in range(2, 7):
                for n_nodes in range(2, 100):
                    G = Waxman_Graph_Container(n_nodes, 0.2, 0.5, max_degree_allowed)
                    self.assertEqual(G.graph.number_of_nodes(), n_nodes)

if __name__ == '__main__':
    unittest.main()