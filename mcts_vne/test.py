import pprint
from agent import Agent
from graph_classes import SN, VNR
from visualization import saveSNImage

def main():
    acc = 0
    sn = SN(n_nodes_min=15, n_nodes_max=15, beta=0.5, alpha=0.4)
    agent = Agent(sn)
    vnr_returned = None
    saveSNImage(agent.substrate_network, './sample/graph{}.png'.format(acc))
    for i in range(20):
        vnr = VNR(n_nodes_min=3, n_nodes_max=4)
        #pp = pprint.PrettyPrinter(indent=4)
        result, vnr_returned = agent.VNoM(vnr)
        if result == 'success_VNoM':
            status, vnr_returned = agent.VLiM_MCF(vnr_returned, False)
            if status == 'success VLiM':
                acc += 1
                saveSNImage(agent.substrate_network, './sample/graph{}.png'.format(acc))
                print(acc)
                pp = pprint.PrettyPrinter(indent=4)
                pp.pprint(vnr_returned.graph.edges(data=True))
    print("successful embeddings:", acc)

if __name__ == "__main__": 
    main()