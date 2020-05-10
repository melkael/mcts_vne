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
    
if __name__ == '__main__':
    unittest.main()