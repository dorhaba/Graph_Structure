#include "Graph.hpp"

int main()
{

    // srand(static_cast<unsigned>(time(0)));

    // Graph g;

    // for (char c = 'A'; c < 'A' + 20; ++c)
    // {
    //     g.addVertex(c, rand() % 10 + 1);
    // }

    // for (int i = 0; i < 50; ++i)
    // {
    //     char src = 'A' + rand() % 20;
    //     char dest = 'A' + rand() % 20;
    //     while (dest == src)
    //     {
    //         dest = 'A' + rand() % 20;
    //     }
    //     double weight = rand() % 20 + 1;
    //     g.addEdge(src, dest, weight);
    // }

    Graph g;

    // Add vertices
    g.addVertex('a', 0.0);
    g.addVertex('b', 0.0);
    g.addVertex('c', 0.0);
    g.addVertex('d', 0.0);
    g.addVertex('e', 0.0);
    g.addVertex('f', 0.0);

    // Add edges
    g.addEdge('a', 'd', -10.0);
    g.addEdge('a', 'b', 10.0);
    g.addEdge('b', 'd', 30.0);
    g.addEdge('d', 'c', 40.0);
    g.addEdge('d', 'e', 20.0);
    g.addEdge('b', 'c', -30.0);
    g.addEdge('c', 'e', 10.0);
    g.addEdge('c', 'f', 20.0);
    g.addEdge('b', 'f', 20.0);
    g.addEdge('e', 'f', 0.0);

    std::cout << "Display Graph" << std::endl;
    g.displayGraph();

    // Get the MST as a tree
    Graph mstTree = g.primMST('a');

    std::cout << std::endl;
    std::cout << "Display mstTree" << std::endl;
    // Display the MST tree
    mstTree.displayGraph();

    mstTree.addEdge('b', 'e', -40.0);

    // Display the MST tree after adding edge
    std::cout << std::endl;
    std::cout << "Display mstTree after adding edge" << std::endl;

    mstTree.displayGraph();

    return 0;
}
