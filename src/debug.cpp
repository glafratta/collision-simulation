 #include "debug.h"

// template <class T>
// void debug::graph_file(const int& it,const T& g, const Disturbance& goal, const std::vector <vertexDescriptor> &plan, const vertexDescriptor& c){
// 	char fileName[50];
// 	sprintf(fileName, "/tmp/graph%04i.txt", it);
// 	FILE * f=fopen(fileName, "w");
// 	auto vs=boost::vertices(g);
// 	for (auto vi=vs.first; vi!=vs.second; vi++){
// 		auto es=boost::out_edges(*vi, g);
// 		if (*vi==c){
// 			fprintf(f,"!");
// 		}
// 		for (vertexDescriptor vp:plan){
// 			if (*vi==vp){
// 				fprintf(f,"*");
// 			}
// 		}
// 		fprintf(f,"%i -> ", *vi);
// 		for (auto ei=es.first; ei!=es.second; ei++){
// 			fprintf(f, "%i (%f) ", (*ei).m_target, g[(*ei)].probability);
// 		}
// 		fprintf(f, "\t(x=%.3f, y= %.3f, theta= %.3f)\n", g[*vi].endPose.p.x, g[*vi].endPose.p.y, g[*vi].endPose.q.GetAngle());
// 	}
// 	fclose(f);
// }

// template <class T>
// void debug::print_graph(const T& g, const Disturbance& goal,const std::vector <vertexDescriptor> &plan, const vertexDescriptor& c){
//     std::stringstream os;
//     auto vs=boost::vertices(g);
//     for (auto vi=vs.first; vi!=vs.second; vi++){
// 		auto es=boost::out_edges(*vi, g);
// 		if (*vi==c){
// 			os<<"!";
// 		}
// 		for (vertexDescriptor vp:plan){
// 			if (*vi==vp){
// 				os<<"*";
// 			}
// 		}
// 		os<<*vi<<"-> ";
// 		for (auto ei=es.first; ei!=es.second; ei++){
// 			os<<(*ei).m_target <<"("<g[(*ei)].probability<<")";
// 		}
// 		os<<"\t(x="<<g[*vi].endPose.p.x<<", y= "<<g[*vi].endPose.p.y<<", theta= "<<g[*vi].endPose.q.GetAngle()<<")\n";
// 	}
//     os.flush();
// }

b2Vec2 GetWorldPoints(b2Body* b, b2Vec2 v){
	b2Vec2 wp=b->GetWorldPoint(v);
	printf("x=%f, y=%f\t", wp.x, wp.y);
}
