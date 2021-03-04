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

#define INFINITE  00000

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
        return NULL;
    }
}



//Function that prints a set of edges belongs breadth tree

void Graph::breadthFirstSearch(ofstream &output_file)
{
    bool visitados[getOrder()]= {};
    visitados[first_node->getId()]=true;
    queue <Node*> nos;

    nos.push(first_node);

    Edge *m=nos.front()->getFirstEdge();

    while(!nos.empty())
    {
        while(m!=NULL)
        {
            m=m->getNextEdge();
            if(!visitados[m->getTargetId()])
            {
                visitados[m->getTargetId()]=true;
                nos.push(getNode(m->getTargetId()));
            }
        }
        cout<<nos.front()->getId()<<"\t";
        output_file<<nos.front()->getId()<<"\t";
        nos.pop();
    }
}




float* Graph::floydMarshall(int idSource, int idTarget)
{

    float mat[getOrder()][getOrder()];

    Node *p=new Node(idSource);

    for(int i=0; p->getId()!=idTarget || p!=NULL; i++)
    {
        for(int j=0; p->getId()!=idTarget || p!=NULL; j++)
        {
            if(i==j)
            {
                mat[i][j]=INFINITE;
            }

            else if(p->hasEdgeBetween(j)!=NULL)
            {
                mat[i][j]=p->hasEdgeBetween(j)->getWeight();
            }

            else
            {
                mat[i][j]=INFINITE;
            }
            p=p->getNextNode();

        }
    }

    for(int i=0; p->getId()!=idTarget || p!=NULL; i++)
    {
        for(int j=0; p->getId()!=idTarget || p!=NULL; j++)
        {
            for(int k=0; p->getId()!=idTarget || p!=NULL; k++)
            {
                if(mat[i][j]>mat[i][k]+mat[k][j])
                {
                    mat[i][j]=mat[i][k]+mat[k][j];
                }

            }
        }
    }


    return *mat;

}



float Graph::dijkstra(int idSource, int idTarget)
{
    float *distancias=new float[getOrder()];
    bool visitados[getOrder()]= {};
    int predecessor[getOrder()];
    Node *p=new Node(idTarget);

    priority_queue < pair<float, Node* >, vector<pair<float, Node* >>, greater<pair<float,Node*  > > > elements;

    for(int i=0; i<this->getOrder(); i++)
    {
        distancias[i]=INFINITE;
        predecessor[i]=-1;
    }

    distancias[idTarget]=0;

    elements.push(make_pair(distancias[idSource],getNode(idTarget)));

    while(!elements.empty())
    {
        pair< float,Node* > data= elements.top();
        Node* atual= data.second;
        elements.pop();

        if(!visitados[atual->getId()])
        {
            for(Edge * m =p->getFirstEdge();p->getId()!=idTarget || m!=NULL; m=m->getNextEdge())
            {
                int ind = m->getTargetId();
                float new_distance = distancias[atual->getId()] + m->getWeight();
                if(distancias[ind] > new_distance)
                {
                    distancias[ind]=new_distance;
                    predecessor[ind] = atual->getId();
                    elements.push(make_pair(distancias[ind],getNode(ind)));
                }
            }
            visitados[atual->getId()]=true;
        }
    }



    return *distancias;
}



void Graph::topologicalSorting()
{
///Verifica Ciclo
    if(verificaCiclo())
    {
        cout<<"O grafo nao é acíclico."<< endl;

    }
    else
    {

///Copia a lista de adjacência
        Node* no;
        vector <int> conjunto;//candidatos
        vector <int> ordem;//solução

        for(no=first_node; no!=NULL; no=no->getNextNode())
        {
            if(no.getInDegree()==0)
                conjunto.push_back(no.getId());
        }
///Ordena
        while(!conjunto.empty())
        {
            ordem.push_back(conjunto.back());
            conjunto.pop_back();
            Node* aux = no(ordem.back()).getNextNode();
            int i = ordem.back();
            while(no != NULL)
            {
                no(i).removeEdge(i,directed,aux);
                if(aux.getInDegree()==0)
                    conjunto.push_back(no(j).getId());
                aux = aux->getNextNode();
            }
        }
///Imprime
        cout << "Ordenacao Topologica: ";
        for(unsigned i=0; i<ordem.size(); i++)
            cout << ordem.at(i) << " ";
        cout << endl;

        delete no;
    }
}



Graph* Graph::getVertexInduced(int listIdNodes[])
{
    int ordem;
    ordem = listIdNodes.size();
    Graph* sub = new Graph(ordem, 0, 0, 0);
    while(!listIdNodes.empty())
    {
        sub->insertNode(listIdNodes.back);
        listIdNodes.pop_back;
    }
    Node* aux1 = sub.getFirstNode();
    Node* aux2 = sub.getFirstNode().getNextNode();
    while(aux1 != NULL)
    {
        while(aux2 != NULL)
        {
            if(aux1.hasEdgeBetween(aux2.getId()) == nullptr)
            {

            }
            else
            {
                sub.insertEdge(aux1->getId(),aux2->getId(),0);
            }
            aux2->getNextNode()

        }

        aux1->getNextNode();
        aux2 = aux1.getNextNode();
    }
    return sub;

}


/*
Graph* agmKuskal()
{
    void agmKuskal(Graph *graph,int orig,int *pai)
    {

        int i,j,dest,primeiro,NV = graph->getNumberEdges;
        double menorPeso;
        int arv =(int) malloc(NV * sizeof(int));
        for(i=0; i<NV; i++)
        {
            arv[i] = i;
            pai[i] = -1;//sem pai
        }
        pai[orig] = orig;
        while(1)
        {
            primeiro=1;
            for(i=0; i<NV; i++) //percorre os vertices
            {
                for(j=0; j < graph->grau[i] ; j++) //arestas
                {
                    //procura vertice menor peso
                    if(arv[i] != arv[graph->arestas[i][j]])
                    {
                        if(primeiro)
                        {
                            menorPeso= graph->pesos [i][j];
                            orig = i;
                            dest = graph->arestas[i][j];
                            primeiro =0;
                        }
                        else
                        {
                            if(menorPeso > graph->pesos[i][j])
                            {
                                menorPeso = graph->pesos[i][j];
                                orig = i;
                                dest = graph->arestas[i][j];
                            }
                        }

                    }
                }
            }
            if(primeiro == 1)
                break;
            if(pai[orig] == -1)
                pai[orig] = dest;
            else
                pai[dest] = orig;

            for(i=0; i<NV; i++)
                if(arv[i] == arv[dest])
                    arv[i] = arv[orig];
        }



    }



}

void agmPrim(Graph *graph,int orig,int *pai)
{

    int i,j,dest,primeiro,NV = graph->getNumberEdges;
    double menorPeso;
    for(i=0; i < NV; i++)
        pai[i] = -1; //sem pai
    pai[orig] = orig;
    while(1)
    {
        primeiro = 1;
        //percorre todos vertices
        for(i=0; i<NV.i++)
        {
            //achou vertices ja visitados
            if(pai[i] != -1)
            {
                //percorre os vizinhos do vertice visitado
                for (j=0; j<graph->grau[i]; j++)
                {
                    if(pai[graph->arestas [i][j]] == -1)
                    {
                        if (primeiro)  //procura aresta de menor custo
                        {
                            menorPeso = graph->pesos[i][j];
                            orig = i;
                            dest = graph->arestas [i][j];
                            primeiro = 0 ;

                        }
                        else
                        {
                            if(menorPeso > graph->pesos[i][j])
                            {
                                menorPeso = graph->pesos[i][j];
                                orig = i;
                                dest = graph->arestas [i][j];
                            }
                        }
                    }
                }
            }
        }
        if(primeiro == 1)
            break;

        pai [dest] = orig;
    }



}

*/
//greed algoritms

 float Graph::greed()
 {


 }

 float Graph::greedRandom()
 {

 }




float Graph::greedRactiveRandom()
{

}


