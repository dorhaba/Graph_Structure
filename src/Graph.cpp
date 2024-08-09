#include "Graph.hpp"
#include <limits>
#include <queue>

// Edge constructor implementation
Edge::Edge(char dest, double wt) : destination(dest), weight(wt) {}

// Vertex constructor implementation
Vertex::Vertex(char id, double wt) : id(id), weight(wt) {}

// Add a vertex to the graph
void Graph::addVertex(char id, double weight)
{
    vertices[id] = Vertex(id, weight);
}

// Add an undirected edge to the graph
void Graph::addEdge(char src, char dest, double weight)
{
    vertices[src].adjacencyList.push_back(Edge(dest, weight));
    vertices[dest].adjacencyList.push_back(Edge(src, weight));
}

// Display the graph
void Graph::displayGraph()
{
    for (const auto &vertexPair : vertices)
    {
        const Vertex &vertex = vertexPair.second;
        std::cout << vertex.id << " --> ";
        for (const Edge &edge : vertex.adjacencyList)
        {
            std::cout << edge.destination << " (Weight: " << edge.weight << "), ";
        }
        std::cout << std::endl;
    }
}

Graph Graph::primMST(char startVertex)
{
    Graph mstTree;

    std::unordered_map<char, bool> inMST;  // A map that tracks whether a vertex is included in the MST (true if it is, false otherwise).
    std::unordered_map<char, double> key;  // A map that stores the minimum weight required to connect each vertex to the growing MST
    std::unordered_map<char, char> parent; // A map that stores the parent vertex for each vertex in the MST. This helps in constructing the tree later.

    for (const auto &vertexPair : vertices)
    {
        char vertex = vertexPair.first;
        key[vertex] = std::numeric_limits<double>::infinity(); // Meaning they are not yet connected to
        inMST[vertex] = false;                                 // Initially, all vertices are marked as not being part of the MST (false).
    }

    std::priority_queue<std::pair<double, char>, std::vector<std::pair<double, char>>, std::greater<std::pair<double, char>>> pq; // min-heap priority queue

    key[startVertex] = 0.0;
    pq.push({0.0, startVertex});
    parent[startVertex] = '\0';
    mstTree.addVertex(startVertex, 0.0); // Add vertices to the new graph

    while (!pq.empty())
    {
        char u = pq.top().second;
        pq.pop();

        inMST[u] = true;

        for (const auto &edge : vertices[u].adjacencyList)
        {
            char v = edge.destination;
            double weight = edge.weight;

            if (!inMST[v] && key[v] > weight)
            {
                key[v] = weight;
                pq.push({key[v], v});
                parent[v] = u;
            }
        }
    }

    // Construct the tree from the parent array
    for (const auto &vertexPair : vertices)
    {
        char v = vertexPair.first;

        if (parent[v] != '\0')
        {
            mstTree.addVertex(v, vertices[v].weight); // Add vertices to the new graph
            mstTree.addEdge(parent[v], v, key[v]);    // Add edges to the new graph
        }
    }

    return mstTree;
}

void Graph::addEdgePrim(char src, char dest, double weight)
{

    vertices[src].adjacencyList.push_back(Edge(dest, weight));
    vertices[dest].adjacencyList.push_back(Edge(src, weight));
}
