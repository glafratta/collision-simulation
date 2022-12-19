#include "src/plan.h"
#include <boost/fusion/container/map.hpp>
#include <boost/fusion/include/map.hpp>
#include<boost/fusion/container/map/map_fwd.hpp>
#include <boost/fusion/include/map_fwd.hpp>
#include <map>

// class State{
// public:
//     enum Outcome {CRASHED, SUCCESS};
//     enum Direction {STRAIGHT, LEFT, RIGHT};
//     struct Obstacle{
//         bool valid;

//         bool isValid(){
//             return valid;
//         }
//     };
//     Outcome outcome;
//     Obstacle obstacle;
//     Direction d;
//     int duration=0;
//     int index =0;

//     Obstacle getObstacle(){
//         return obstacle;
//     }
    
// };


//STATE SIZE 784 OBJECT 136 

//use std::map?

int main(){
    // State::Object o(ObjectType::obstacle, {0.1, 0.0f});
    // State::Object o2(ObjectType::obstacle, {0.2, 0.0f});
    State task;
    State left;
    State right;
    //boost::fusion::map <pretendState *, pretendState*, pretendState*> m; //works with no <type, type>

    Tree::Node n;
    // n.setRoot(&state);
    n.addEdge(static_cast<Tree::Node::nodeType> (State::Direction::LEFT), &left);
    // n.setRight(&right);


}