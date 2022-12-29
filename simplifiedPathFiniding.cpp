#include "src/state.h"
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp> //adjacency list
#include <boost/graph/depth_first_search.hpp>
//new includes
#include "boost/tuple/tuple.hpp" //boost::tie
#include "boost/tuple/tuple_comparison.hpp"
#include "boost/tuple/tuple_io.hpp"
using namespace boost;

typedef adjacency_list<vecS, vecS, boost::bidirectionalS, State, int> Graph;

typedef std::pair<State &, State&> Edge; //can
typedef graph_traits<Graph>::vertex_iterator vIt;
typedef graph_traits<Graph>::vertex_descriptor vD;
typedef graph_traits<Graph>::edge_descriptor eD;
typedef graph_traits<Graph>::edge_iterator eIt;


bool isFullLength(vD v, Graph &g, int length=0){
    if (in_degree(v,g)<=0 && length <5){
        return false;
    }
    else if (length >=5){
        return true;
    }
    else{
        eD backEdge= in_edges(v, g).first.dereference();
        length += g[backEdge];
        vD newV = source(backEdge, g);
        return isFullLength(newV, g, length);
    }

}


    vD addV(vD v, Graph &g){  //this is a simplified eliminatedisturbance
        printf("add to vertex %i, ", v);
        //get in-edge to assign a weight to it
        eD e;
        vD srcV;
        bool backEdgeExists = boost::in_degree(v, g)>0;
        int sourceOutDegree =-1;
        vD v1 = v;
        if (backEdgeExists){ //check edges coming in
            e = in_edges(v, g).first.dereference(); //get edge descriptor for in-edge
            g[e]=1;
            srcV = boost::source(e, g);
            printf("src = %i\n", srcV);
            sourceOutDegree = boost::out_degree(srcV, g);

        }
        if (g[v].obstacle.isValid()){
            if (backEdgeExists){ //add weight to back edge
                g[e]= 2;
            }
            //CHECK ALL BACK EDGES
            if (!isFullLength(v, g)){ //if max length has not been reached, add an avoidance state
            printf("not full length\n");
                    v1 =add_vertex(g);
                    g[v1] = State();
                    add_edge(v, v1, g).first;
                    printf("added edge %i, %i\n", v, v1);
            }
             //this needs to be outside obstacle valid because it should be done regardless of whether v collides, if the paht is over, move on to next branch

        }
        else{
            printf("obstacle not valid\n");
        }

        if (isFullLength(v, g)){
            printf("full length, adding nodes\n");
            if (sourceOutDegree ==1) { //add another vertex to source node
            v1 =add_vertex(g);
            printf("source degree 1, src = %i\n", srcV);
            g[v1] = State();
            add_edge(srcV, v1, g).first;
            printf("added edge %i, %i\n", srcV, v1);
            //addV(v1, g);
            }
            else if (sourceOutDegree==2){ //end of the branch
                while (sourceOutDegree>=2){
                    printf("new src = %i, in degree = %i\n", srcV, boost::in_degree(srcV, g));
                    if(boost::in_degree(srcV, g)>0){
                        eD srcBackEdge = boost::in_edges(srcV, g).first.dereference();
                        vD nextIncompleteVertex = source(srcBackEdge, g);
                        sourceOutDegree= boost::out_degree(nextIncompleteVertex,g);
                        printf("source degree %i src= %i\n", sourceOutDegree, nextIncompleteVertex);
                        auto oe = out_edges(nextIncompleteVertex,g);
                        for (oe.first; oe.first!=oe.second; oe.first++){
                            printf("edge from %i to %i\n", oe.first.dereference().m_source, oe.first.dereference().m_target);
                        }
                        srcV = nextIncompleteVertex;
                    }
                    else{
                        printf("source has no back edge\n");
                        break;
                    }
                }
                if (sourceOutDegree <2){
                    v1 =add_vertex(g);
                    //auto v2 = add_vertex(g);
                    g[v1] = State();
                    //g[v2] = State();
                    add_edge(srcV, v1, g).first;
                    printf("added edge %i, %i\n", srcV, v1);
                    //addV(v1, g);
                }
            }

        }
        return v1;
    }


void build_tree(vD v, Graph&g){
    std::vector <vD> listCollided = {0, 1, 2, 5, 8, 12, 9};
    printf("v=%i\n", v);
    for (vD d: listCollided){
        if (v==d){
            g[v].setObstacle(State::Object(ObjectType::obstacle, b2Vec2(0.12, 0)));
            printf("added ob to %i\n", v);
        }
    }
    vD v1 = addV(v, g);
    if (v1!=v){
        build_tree(v1, g);
    }

}






 int main(){
    State s1;
    Graph tree;

    auto v0 = add_vertex(tree);
    tree[0]=s1;
    build_tree(0, tree);

    printf("tree size = %i\n", tree.m_vertices.size());

 }