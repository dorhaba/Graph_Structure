#include "Graph.hpp"
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <functional>

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

/*
1. Check if the new edge forms a cycle in the MST.
   If it doesn't, the edge can be added without affecting the MST.

2. If the new edge does form a cycle, then check if the new edge has a smaller weight than any other edge in the cycle.
   If it does, replace the heaviest edge in the cycle with the new edge to maintain the minimum spanning tree property.

*/

// DFS-based helper function for cycle detection
std::vector<std::pair<char, char>> Graph::findCycleInMST(char startVertex)
{
    std::unordered_map<char, char> parent;
    std::unordered_map<char, bool> visited;
    std::vector<std::pair<char, char>> cycleEdges;

    std::function<bool(char, char)> dfs = [&](char vertex, char parentVertex) -> bool
    {
        visited[vertex] = true;
        parent[vertex] = parentVertex;

        for (const Edge &edge : vertices[vertex].adjacencyList)
        {
            if (!visited[edge.destination])
            {
                if (dfs(edge.destination, vertex))
                {
                    if (cycleEdges.empty() || edge.destination != startVertex)
                    {
                        cycleEdges.push_back({vertex, edge.destination});
                    }
                    return true;
                }
            }
            else if (edge.destination != parentVertex)
            {
                startVertex = edge.destination;
                cycleEdges.push_back({vertex, edge.destination});
                return true;
            }
        }
        return false;
    };

    visited.clear();
    dfs(startVertex, '\0');

    return cycleEdges;
}

void Graph::removeEdge(char src, char dest)
{
    // Lambda function to find the edge to remove
    auto removeEdgeFromList = [dest](const Edge &edge)
    { return edge.destination == dest; };

    // Remove edge from src to dest
    vertices[src].adjacencyList.erase(
        std::remove_if(vertices[src].adjacencyList.begin(), vertices[src].adjacencyList.end(), removeEdgeFromList),
        vertices[src].adjacencyList.end());

    // Remove edge from dest to src
    vertices[dest].adjacencyList.erase(
        std::remove_if(vertices[dest].adjacencyList.begin(), vertices[dest].adjacencyList.end(), removeEdgeFromList),
        vertices[dest].adjacencyList.end());
}

void Graph::addEdgePrim(char src, char dest, double weight)
{
    addEdge(src, dest, weight);

    // Detect the cycle using DFS

    std::vector<std::pair<char, char>> cycleEdges = findCycleInMST(src);

    // If no cycle was detected, no update is needed
    if (cycleEdges.empty())
        return;

    // If no cycle was detected, no update is needed
    if (cycleEdges.empty())
        return;

    // Identify the heaviest edge in the cycle
    std::pair<char, char> maxEdge;
    double maxWeight = -1.0;

    for (const auto &edgePair : cycleEdges)
    {
        char u = edgePair.first;
        char v = edgePair.second;
        double w = 0.0;

        // Find the corresponding edge weight
        for (const Edge &edge : vertices[u].adjacencyList)
        {
            if (edge.destination == v)
            {
                w = edge.weight;
                break;
            }
        }

        if (w > maxWeight)
        {
            maxWeight = w;
            maxEdge = edgePair;
        }
    }

    // If the new edge is lighter, remove the heaviest edge in the cycle
    if (weight < maxWeight)
    {
        removeEdge(maxEdge.first, maxEdge.second);
    }
    else
    {
        // If the new edge is not lighter, remove it from the MST
        removeEdge(src, dest);
    }
}
