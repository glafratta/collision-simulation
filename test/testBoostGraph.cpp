#include "../src/state.h"
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
//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, State> GraphA;

typedef std::pair<State &, State&> Edge; //can
typedef graph_traits<Graph>::vertex_iterator vIt;
typedef graph_traits<Graph>::vertex_descriptor vD;
typedef graph_traits<Graph>::edge_descriptor eD;
//typedef graph_traits<Graph>::out_edge_iterator eIt;
typedef graph_traits<Graph>::edge_iterator eIt;
//template <class Graph, class P, class T, class R>;
//void depth_first_visit(Graph& G, const bgl_named_params<P, T, R>& params);



// class Graph2: public adjacency_list<vecS, vecS, bidirectionalS, State>{
// public:
//     State state;

//     Graph2(){}

//     Graph2(State & s):state(s){}

//     void prune(vIt index){


//     }
// };

// class DefaultVisitor: public default_dfs_visitor{
//     public:
//     template <typename V, typename G>
//     void discover_vertex(V, G&) { } //we use

//     template <typename E, typename G>
//     void tree_edge(E, const G&) { }

//     template <typename E, typename G>
//     void back_edge(E, const G&) { } //we use

//     template <typename E, typename G>
//     void forward_or_cross_edge(E, const G&) { }

//     template <typename V, typename G>
//     void finish_vertex(V, const G&) { }

//     template <typename V, typename G>
//     void start_vertex(V, const G&){}

// };

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
        //std::vector <eD> newEs;
        if (g[v].obstacle.isValid()){
            if (backEdgeExists){ //add weight to back edge
                g[e]= 2;
            }
            //CHECK ALL BACK EDGES
            if (!isFullLength(v, g)){ //if max length has not been reached, add an avoidance state
            printf("not full length\n");
                //for (int i =0; i<2; i++){
                    v1 =add_vertex(g);
                    //auto v2 = add_vertex(g);
                    g[v1] = State();
                    //g[v2] = State();
                    add_edge(v, v1, g).first;
                    printf("added edge %i, %i\n", v, v1);
                    //newEs.push_back(e1);
                    //auto e2 = add_edge(v, v2, g);
                //}
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
        //return newEs;
    }


void build_tree(vD v, Graph&g){
    std::vector <vD> listCollided = {0, 1, 2, 5, 12, 9};
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

int forward(vD v){ //next branch in the subtree
    v=v+1;
    if (v<0){
        v=-1;
    }
    return v;
}

int back(vD v){ //
    v = v-2;
    if (v<0){
        v=-1;
    }
    return -1;
}




// class TreeBuilder:public default_dfs_visitor{ //tried and failed depth_first visit. sequence is 0-1-3-4-2-7-8- double free or corruption
//     public:
//     Graph& graph; //keep graph in memory
//     std::vector <vD> listCollided = {0, 1, 2, 5, 12, 9};
//     TreeBuilder(Graph & g): graph(g){}
//     // std::vector<boost::default_color_type> colors(size_t(3)); //make vector with colours
//     // boost::iterator_property_map colorMap(colors.begin(), boost::get(boost::vertex_index, graph));


//     void discover_vertex(vD v, const Graph & g) {
//         //printf("started\n");
//         graph = (g);
//         printf("v = %i\n", v);
//         for (vD d: listCollided){
//             if (v==d){
//                 graph[v].setObstacle(State::Object(ObjectType::obstacle, b2Vec2(0.12, 0)));
//                 printf("added ob\n");
//             }
//         }

//         // eD e;
//         // vD srcV;
//         // bool backEdgeExists = boost::in_degree(v, g)>0;
//         // int sourceOutDegree =-1;
//         // if (backEdgeExists){ //check edges coming in
//         //     e = in_edges(v, graph).first.dereference(); //get edge descriptor for in-edge
//         //     graph[e]=1;
//         //     srcV = boost::source(e, graph);
//         //     printf("src = %i\n", srcV);
//         //     sourceOutDegree = boost::out_degree(srcV, graph);

//         // }
//         addV(v,graph); //graph=
//         // printf("graph = %i, g=%i\n", graph.m_vertices.size(), g.m_vertices.size());
//         // if (isFullLength(v, graph)){
//         // printf("full length, adding nodes\n");
//         //     if (sourceOutDegree ==1) { //add another vertex to source node
//         //     auto v1 =add_vertex(graph);
//         //     printf("source = %i\n", srcV);
//         //     graph[v1] = State();
//         //     add_edge(srcV, v1, graph).first;
//         //     printf("added edge %i, %i\n", srcV, v1);
//         //     //addV(v1, graph);
//         //     //discover_vertex(v1, g);
//         //     //depth_first_visit(g, v1, *(this));
//         //     }
//         //     else if (sourceOutDegree ==2){
//         //         if(boost::in_degree(srcV, graph)>0){
//         //             eD srcBackEdge = boost::in_edges(srcV, graph).first.dereference();
//         //             vD srcSrcV = source(srcBackEdge, graph);
//         //             printf("source = %i\n", srcV);
//         //             auto v1 =add_vertex(graph);
//         //             //auto v2 = add_vertex(g);
//         //             graph[v1] = State();
//         //             //g[v2] = State();
//         //             add_edge(srcSrcV, v1, graph).first;
//         //             printf("added edge %i, %i\n", srcSrcV, v1);
//         //             //addV(v1, graph);
//         //             //discover_vertex(v1, g);
//         //             //depth_first_visit(g, v1, *(this));

//         //         }
//         //         else{
//         //             printf("source has no back edge\n");
//         //         }
//         // }

// }






//     void back_edge(eD e, const Graph & g){
//        printf("back edge between %i, %i\n", e.m_source, e.m_target);

//     }

//     void finish_vertex(vD v, const Graph& g){
//         printf("finish vertex: %i\n", v);
//     }

//     void printGraphSize(){
//         printf("graph size = %i\n", graph.m_vertices.size());
//     }

//     void tree_edge(eD e, const Graph &g){
//        printf("tree edge between %i, %i\n", e.m_source, e.m_target);
//     }

//     void examine_edge(eD e, const Graph& g){
//        printf("examine edge between %i, %i\n", e.m_source, e.m_target);

//     }

//     void start_vertex(vD v, const Graph &g){
//        printf("start vertex %i, %i\n", v);

//     }
//     //Graph & getGraph()const{return graph;}
// };





 int main(){
//     //CREATE STATES and add to tree
//     std::vector <State> states;
    State s1,s2,s3;
    s1.accumulatedError = 100; //this should not be included in the calculation
    //s2.accumulatedError = 200;
    //s3.accumulatedError =0;
//     states.push_back(s1);
//     states.push_back(s2);
//     states.push_back(s3);

//     Tree tree;
//     Tree::Node node;
//     // node.init(&s1);
//     // node.addChild(s2);
//     // node.addChild(s3);

//     // std::map<Tree::Node*, float> distances = tree.getDistanceAtLeaves();
//     // printf("duration s2 = %f", distances.find(&s2)->second);
//     //printf("duration s3 = %f", distances.find(&s3)->second);

//Edge e(s1, s2);
Graph tree;

auto v0 = add_vertex(tree);
// add_vertex(tree);
tree[0]=s1;
// tree[1]=s2;
// tree[2]=s3;
// auto v = add_vertex(tree);
// //printf("duration = %i\n", tree[0].stepDuration);
// auto e =add_edge(0, 1, tree);
// auto e2 =add_edge(0, 2, tree);
// tree[e.first] =0;
// auto e3 =add_edge(2,3, tree);
// tree[e2.first]=0;
// tree[e3.first]=4;
// std::pair<vIt, vIt> p = vertices(tree);

//printf("v = %i\n", v); //returns index
//add_edge(e.first, e.second, tree);
// printf("first edge is between %i, %i\n", e.first.m_source, e.first.m_target);
//WHAT HAPPENS WHEN A VERTEX IS DELETED
// printf("removing tree[0]\n");
// remove_vertex(0, tree);
//printf("are the values shifted back? tree[0] old duration was 100, new one is %i\n", tree[0].stepDuration);
//auto ei = edges(tree);
//printf("first edge is between %i, %i after removing 0\n\n\n", e.first.m_source, e.first.m_target);

//EXPLORE EDGES
// printf("iteration through edges\n");
// std::pair <vIt, vIt> es= vertices(tree);
// auto ee = edges(tree);
// for (ee.first; ee.first!=ee.second; ee.first++){
//     //printf("first edge is between %i, %i\n", es.first->m_source, es.first->m_target);
//     vD d = *es.first; //returns iterator
//     printf("testing d = %f\n", tree[d].accumulatedError);
//     printf("created descript\n");
//     eD eit = *ee.first; //we want to enter vertex descriptor
//     //eD ed = *eit;
//     //auto out = tree.out_edge_list();
//     printf("created edge descript\n");
//     printf("edge properties\nvalue of edge: %i\n", tree[eit]);
// }

// //test accessibility
// printf("now adding a vertex");
// addBranch(tree, v);
// for (ee.first; ee.first!=ee.second; ee.first++){
//     printf("edge is between %i, %i\n", ee.first->m_source, ee.first->m_target);
// }
//printf("duration = %i\n",tree[5].stepDuration);

//what happens if an extreme of an edge is null
// add_vertex(tree);
// auto f = add_edge(5,6,tree ).first;
// printf("is last element initialised?? acc error = %f\n",tree[f]); //State default onstructor is invoked

//TreeBuilder tb(tree);
tree[0].setObstacle(State::Object(ObjectType::obstacle, b2Vec2(0.12, 0)));
// std::vector<boost::default_color_type> colors(14); //make vector with colours
// // for (auto c: colors){
// //     printf("color %i\n", c);
// // }
// boost::iterator_property_map colorMap(colors.begin(), boost::get(boost::vertex_index, tree));

// depth_first_visit(tree, 0, tb, colorMap);

//tb.printGraphSize();
//tree = tb.graph;

build_tree(0, tree);

printf("tree size = %i\n", tree.m_vertices.size());

printf("\n\n\nTEST\n\n\n");
Graph test;
vD b = add_vertex(test);
auto a = add_edge(b, b, test).first;
test[a] = 2;
auto c = boost::in_edges(b, test).first.dereference();
printf("c between %i, %i, value = %i", c.m_source, c.m_target, test[c]);


//TEST boost::get
// vD v1 =add_vertex(tree);
// tree[1]=s2;
// auto ed= add_edge(0, 1, tree).first;
// tree[ed]=1;
// printf("1. is full length? %i\n", isFullLength(v1, tree));
// vD v2 = add_vertex(tree);
// tree[2]=s2;
// auto e2 =add_edge(0, 2, tree);
// tree[e2.first] =6; //full length
// printf("2. is full length? %i\n", isFullLength(v2, tree));
// vD v3 =add_vertex(tree);
// tree[3]=s3;
// auto e3 =add_edge(1,3, tree).first;
// tree[ed]=4;
// printf("3. is full length?%i\n", isFullLength(v2, tree));

// addV(0, tree);
// printf("addV n vertices = %i\n", tree.m_vertices.size());
// vD v =0;
// auto t = in_edges(v, tree);
// auto deg = boost::in_degree(v, tree);
// //eD eIn = *(boost::in_edges(v, tree).first);
// printf("ed = %i\t degree = %iin-edge sourc for v=\n", t.first.m_src, deg);
// printf("test dereferencing with degree 1, source = %i, target = %i", t.first.dereference().m_source, t.first.dereference().m_target);
 }
