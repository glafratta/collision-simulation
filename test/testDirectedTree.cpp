#include <stdio.h>
#include "../src/general.h"
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Node, Edge> undirectedTree;


int main(){
    Tree t;
    CollisionGraph g;
    undirectedTree u;
    vertexDescriptor v= boost::add_vertex(g);
    vertexDescriptor v1= boost::add_vertex(t);
    vertexDescriptor v2 = boost::add_vertex(u);
    printf("out edges t[0] = %i", boost::out_degree(v1,t));
    printf("in edges g[0] = %i", boost::in_degree(v, g));
    printf("out edges g[0] = %i", boost::out_degree(v2, u));

}

