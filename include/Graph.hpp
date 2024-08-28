#pragma once

#include <iostream>
#include <unordered_map>
#include <vector>
#include <utility>

class Edge
{
public:
    char destination;
    double weight;

    Edge(char dest, double wt);
};

class Vertex
{
public:
    char id;
    double weight;
    std::vector<Edge> adjacencyList;

    // Default constructor
    Vertex() : id('\0'), weight(0.0) {}

    Vertex(char id, double wt);
};

class Graph
{
private:
    std::unordered_map<int, Vertex> vertices;
    std::vector<std::pair<char, char>> findCycleInMST(char startVertex);

public:
    void removeEdge(char src, char dest);
    void addVertex(char id, double weight);
    void addEdge(char src, char dest, double weight);
    void displayGraph();
    Graph primMST(char startVertex);
    void addEdgePrim(char src, char dest, double weight);
};
