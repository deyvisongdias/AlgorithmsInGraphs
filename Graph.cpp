#include "Graph.h"
#include "Node.h"
#include "Edge.h"
#include <iostream>
#include <fstream>
#include <stack>
#include <queue>
#include <list>
#include <math.h>
#include <cstdlib>
#include <ctime>
#include <float.h>
#include <iomanip>

using namespace std;

/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

// Constructor
Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node)
{

    this->order = order;
    this->directed = directed;
    this->weighted_edge = weighted_edge;
    this->weighted_node = weighted_node;
    this->first_node = this->last_node = nullptr;
    this->number_edges = 0;
}

// Destructor
Graph::~Graph()
{

    Node *next_node = this->first_node;

    while (next_node != nullptr)
    {

        next_node->removeAllEdges();
        Node *aux_node = next_node->getNextNode();
        delete next_node;
        next_node = aux_node;
    }
}

// Getters
int Graph::getOrder()
{

    return this->order;
}
int Graph::getNumberEdges()
{

    return this->number_edges;
}
//Function that verifies if the graph is directed
bool Graph::getDirected()
{

    return this->directed;
}
//Function that verifies if the graph is weighted at the edges
bool Graph::getWeightedEdge()
{

    return this->weighted_edge;
}

//Function that verifies if the graph is weighted at the nodes
bool Graph::getWeightedNode()
{

    return this->weighted_node;
}


Node *Graph::getFirstNode()
{

    return this->first_node;
}

Node *Graph::getLastNode()
{

    return this->last_node;
}

// Other methods
/*
    The outdegree attribute of nodes is used as a counter for the number of edges in the graph.
    This allows the correct updating of the numbers of edges in the graph being directed or not.
*/
void Graph::insertNode(int id)
{
    Node *p;
    if(first_node==NULL)
    {
        p=new Node(id);
        first_node=p;
        last_node=p;
    }
    else
    {
        p=new Node(id);
        last_node->setNextNode(p);
        last_node=p;
    }
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    if(id ==target_id)//
    {
        return;
    }
    if(searchNode(id)==false)
    {
        insertNode(id);
    }
    if(searchNode(target_id)==false)
    {
        insertNode(target_id);
    }
    Node *aux;
    aux=getNode(id);

    if(aux->searchEdge(target_id)==false)
    {
       aux->insertEdge(target_id,0);
       aux->incrementInDegree();
       aux->incrementOutDegree();

       aux=getNode(target_id);
       aux->incrementInDegree();
       aux->incrementOutDegree();

       aux->insertEdge(id,0);
       number_edges++;
    }

}



void Graph::removeNode(int id)
{
    /*
    Node *p;
    for(p=first_node; p!=NULL; p=p->getNextNode())
    {
        if(p->getId()==id)
        {


        }
    */
}

bool Graph::searchNode(int id)
{
    Node *p;
    for(p=first_node; p!=NULL; p=p->getNextNode())
    {
        if(p->getId()==id)
        {
            return true;
        }
    }
    return false;
}

Node *Graph::getNode(int id)
{
    Node *p;
    for(p=first_node; p!=NULL; p=p->getNextNode())
    {
        if(p->getId()==id)
        {
            return p;
        }
        return NULL
    }
}



//Function that prints a set of edges belongs breadth tree

void Graph::breadthFirstSearch(ofstream &output_file)
{

}



float Graph::floydMarshall(int idSource, int idTarget)
{

}



float Graph::dijkstra(int idSource, int idTarget)
{

}

//function that prints a topological sorting
void topologicalSorting()
{

}

void breadthFirstSearch(ofstream& output_file)
{

}

Graph* getVertexInduced(int* listIdNodes)
{

}

Graph* agmKuskal()
{

}

Graph* agmPrim()
{

}

