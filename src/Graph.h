/**************************************************************************************************
 * Implementation of the TAD Graph
**************************************************************************************************/

#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED
#include "Node.h"
#include <fstream>
#include <stack>
#include <list>

using namespace std;

class Graph{

    //Atributes
    private:
        int order;
        int number_edges;
        bool directed;
        bool weighted_edge;
        bool weighted_node;
        Node* first_node;
        Node* last_node;

    public:
        //Constructor
        Graph(int order, bool directed, bool weighted_edge, bool weighted_node);
        //Destructor
        ~Graph();
        //Getters
        int getOrder();
        int getNumberEdges();
        bool getDirected();
        bool getWeightedEdge();
        bool getWeightedNode();
        Node* getFirstNode();
        Node* getLastNode();
        //Other methods
        void insertNode(int id);
        void insertEdge(int id, int target_id, float weight);
        void removeNode(int id);
        bool searchNode(int id);
        Node* getNode(int id);

        //methods phase1
        void topologicalSorting();
        void breadthFirstSearch(ofstream& output_file);
        Graph* getVertexInduced(int* listIdNodes);
        void agmKuskal(ofstream& output);
        void agmPrim(ofstream &output_file, int id);
        float* floydMarshall(int idSource, int idTarget);
        float dijkstra(int idSource, int idTarget);

        //methods phase1
        void greed(ofstream& output_file);

        float greedRandom();
        //Auxiliar methods
       float auxGreed(ofstream& output_file);
};

#endif // GRAPH_H_INCLUDED
