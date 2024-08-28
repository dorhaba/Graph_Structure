#include "Graph.hpp"

// 322876533 עידו שמרלינג
// 211678818 דור חבסוב

int main()
{
    Graph graph;

    // Add 20 vertices labeled from 'A' to 'T'
    for (char id = 'A'; id <= 'T'; ++id)
    {
        graph.addVertex(id, 0.0);
    }

    // Add 50 edges with predefined weights
    graph.addEdge('A', 'B', 5);
    graph.addEdge('A', 'C', 30);
    graph.addEdge('A', 'D', 18);
    graph.addEdge('B', 'C', 7);
    graph.addEdge('B', 'E', -2);
    graph.addEdge('C', 'F', 6);
    graph.addEdge('D', 'G', -4);
    graph.addEdge('E', 'F', 15);
    graph.addEdge('E', 'H', 9);
    graph.addEdge('F', 'I', 22);
    graph.addEdge('G', 'J', 52);
    graph.addEdge('H', 'I', 7);
    graph.addEdge('I', 'K', 14);
    graph.addEdge('J', 'L', 3);
    graph.addEdge('K', 'M', 8);
    graph.addEdge('L', 'N', 6);
    graph.addEdge('M', 'O', 2);
    graph.addEdge('N', 'P', 9);
    graph.addEdge('O', 'Q', 4);
    graph.addEdge('P', 'R', 7);
    graph.addEdge('Q', 'S', -5);
    graph.addEdge('R', 'T', 1);
    graph.addEdge('S', 'T', 3);
    graph.addEdge('A', 'E', 2);
    graph.addEdge('B', 'F', 64);
    graph.addEdge('C', 'G', -5);
    graph.addEdge('D', 'H', -8);
    graph.addEdge('E', 'I', 4);
    graph.addEdge('F', 'J', -3);
    graph.addEdge('G', 'K', 7);
    graph.addEdge('H', 'L', 90);
    graph.addEdge('I', 'M', 1);
    graph.addEdge('J', 'N', -6);
    graph.addEdge('K', 'O', -5);
    graph.addEdge('L', 'P', -8);
    graph.addEdge('M', 'Q', 3);
    graph.addEdge('N', 'R', 7);
    graph.addEdge('O', 'S', -4);
    graph.addEdge('P', 'T', 2);
    graph.addEdge('Q', 'A', -9);
    graph.addEdge('R', 'B', 6);
    graph.addEdge('S', 'C', 4);
    graph.addEdge('T', 'D', 7);
    graph.addEdge('A', 'F', 3);
    graph.addEdge('B', 'G', 8);

    std::cout << "Display Graph" << std::endl;
    graph.displayGraph();

    // Get the MST as a tree
    Graph mstTree = graph.primMST('A');

    std::cout << std::endl;
    std::cout << "Display mstTree" << std::endl;
    // Display the MST tree
    mstTree.displayGraph();

    mstTree.addEdgePrim('P', 'Q', -40.0);

    // Display the MST tree after adding edge
    std::cout << std::endl;
    std::cout << "Display mstTree after adding edge" << std::endl;

    mstTree.displayGraph();

    return 0;
}
