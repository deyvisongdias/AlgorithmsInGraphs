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
            for(Edge * m =p->getFirstEdge(); p->getId()!=idTarget || m!=NULL; m=m->getNextEdge())
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




void Graph::prim(ofstream &output_file, int id)
{
    priority_queue < pair<float, Node* >, vector<pair<float, Node* >>, greater<pair<float,Node*  > > > data_edges; ///Fila de prioridade que ordena os nós de acordo com a distancia em relacao ao anterior
    vector <float> distances(order,INFINITE); ///Vetor de distancias
    vector <int> predecessor(order,-1);   ///Vetor antecessores
    bool visited[order]= {};     ///Vetor de visitados
    Node *p=getNode(id); ///Obtem o no que sera a raiz da arvore
    data_edges.push(make_pair(0,p)); ///Adiciona o nó inicial na fila e marca sua distancia como zero
    distances[p->getId()]=0;///Distancia para o nó inicial é marcada como zero

    while(!data_edges.empty()) ///Enquanto a fila nao estiver vazia
    {
        Node *d = data_edges.top().second; ///Pega o nó do topo da fila
        data_edges.pop();  ///Remove o topo da fila
        visited[d->getId()]=true; /// Marca o nó como visitado
        for(Edge * q= d->getFirstEdge() ; q!=nullptr ; q=q->getNextEdge())///Percorre a lista de arestas
        {
            if(!visited[q->getTargetId()] &&  q->getWeight() < distances[q->getTargetId()]) ///Se o no alvo nao foi visitado && o peso atual é menor que o peso antigo
            {
                distances[q->getTargetId()]=q->getWeight(); ///Atualiza a distancia/peso
                data_edges.push(make_pair(q->getWeight(),getNode(q->getTargetId()))); ///Insere na fila o proximo nó e a distancia ate ele
                predecessor[q->getTargetId()] = d->getId(); ///Atualiza o antecessor do nó
            }
        }
    }
    float total_cost=0;///Somario dos pesos das arestas
    cout<<"Arestas:\n";
    output_file <<"Arestas:\n";
    for (int i = 0; i < order; ++i)
    {
        if(i!=id)
        {
            cout<<predecessor[i]<<" - "<< i <<endl;///Mostra a aresta de i para j
            output_file<<predecessor[i]<<" - "<< i <<endl;///Grava a aresta de i para j
            total_cost+=distances[i];///Incrementa o custo
        }

    }
    cout<<"Custo da AGM: "<<total_cost<<endl;
    output_file <<"Custo da AGM: "<<total_cost<<endl;
}

void Graph::kruskal(ofstream& output) ///algoritmo gerador de arvore minima: kruskal, recebe o arquivo de saida para gerar a saida
{
    list<Edge*> arestas; ///cria uma lista que armazenara todas as arestas do grafo
    vector<Edge*> tree; ///cria um vetor de arestas que armazenará a solução
    Node *p;
    Edge *q;
    Edge *qaux;
    float soma=0; ///variavel que armazenara o somatorio das arestas da solução
    for(p=this->getFirstNode(); p!=nullptr; p=p->getNextNode()) ///percorre a lista de nós do grafo
    {
        for(q=p->getFirstEdge(); q!=nullptr; q=q->getNextEdge()) ///percorre a lista de arestas do grafo
        {
            arestas.push_back(q); ///adiciona a aresta na lista

            if(this->getNode(q->getTargetId())->hasEdgeBetween(p->getId())!=nullptr) ///verifica se a aresta adicionada já não havia sido adicionada previamente
                ///acessando o nó alvo e verificando se a aresta ja havia sido inserida
            {
                qaux=this->getNode(q->getTargetId())->hasEdgeBetween(p->getId()); ///pega a aresta que une o alvo e o id atual
                std::list<Edge*>::iterator find_aresta_2=std::find(arestas.begin(),arestas.end(),qaux);///pega a referencia à aresta ou pega o ultimo elemento da lista
                if(find_aresta_2!=arestas.end())   ///caso a aresta não seja a ultima ele a remove da lista de arestas
                    arestas.pop_back();
            }
        }
    }
    arestas.sort([](Edge * a1, Edge * a2)
    {
        return a1->getWeight() < a2->getWeight();
    }); ///organiza as arestas por ordem crescente atravez de comparação
    int *subset = new int[this->order]; ///cria vetor com tamanho da ordem do grafo,atravez dele sera possivel
    ///verificar se as arestas a serem inseridas formam ciclos com as ja dispostas na solução
    for(int i=0; i<this->order; i++) ///inicializa o vetor subset com -1
    {
        subset[i]=-1;
    }
    for(int i=0; i<this->getNumberEdges(); i++) ///usa um contador para executar a tarefa para cada numero de arestas do grafo
    {
        int aux_v1=search_edge(arestas.front(),arestas.front()->getTargetId()); ///pega o id do nó de "origem" da aresta de menor peso da lista (front)
        int aux_v2=arestas.front()->getTargetId(); ///armazena o id do nó "alvo" da aresta de menor peso da lista (front)
        int v1=buscar(subset,aux_v1);  ///armazena o indice para o subconjunto ao qual o id v1 pertence
        int v2=buscar(subset,aux_v2); ///armazena o indice para o subconjunto ao qual o id v2 pertence
        if(v1!=v2)                   ///se os subconjuntos forem diferentes ele os une
        {
            cout << aux_v1  << " - " << aux_v2 <<" " <<  arestas.front()->getWeight()  << endl;    ///imprime conexões dos vertices
            output << aux_v1  << " - " << aux_v2 <<" " <<  arestas.front()->getWeight()  << endl; ///grava no arquivo de saida as conexões de vertice
            soma=soma+arestas.front()->getWeight(); ///soma dos pesos das arestas
            tree.push_back(arestas.front()); ///armazena as arestas em uma arvore
            unir(subset,v1,v2);
        }
        arestas.pop_front();
    }
    cout << soma << endl; ///imrpime a soma dos pesos das arestas da arvore
    output  << soma << endl;
}


*/
//greed algoritms

void Graph::greed(ofstream& output_file)
{

    auto start =  chrono::high_resolution_clock::now();//inicia a contagem do tempo

    vector<Node*> Arvore=auxGreed(ofstream& output_file);

    auto end= chrono::high_resolution_clock::now() - start;
    long long nanoseconds =chrono::duration_cast<chrono::nanoseconds>(end).count();
    cout << nanoseconds*pow(10,-6) << endl;
    output_file <<"Tempo(ms): "<< nanoseconds*pow(10,-6) << endl;
    // output_file << "Qualidade da Solucao( Maior grau ) : "<<s.size() <<endl;


}


float Graph::greedRandom()
{











}





