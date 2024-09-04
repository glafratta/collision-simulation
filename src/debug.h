#ifndef DEBUG_H
#define DEBUG_H

#include "worldbuilder.h"
#include <fstream>

namespace debug{
	
// template <class T>
// void graph_file(const int &, const T&,const Disturbance &, std::vector <vertexDescriptor>,const vertexDescriptor&);

template <class T>
void print_graph(const T& g, const Disturbance & goal, std::vector <vertexDescriptor>plan, const vertexDescriptor& c){
     std::stringstream os;
    auto vs=boost::vertices(g);
    for (auto vi=vs.first; vi!=vs.second; vi++){
		auto es=boost::out_edges(*vi, g);
		if (*vi==c){
			os<<"!";
		}
		for (vertexDescriptor vp:plan){
			if (*vi==vp){
				os<<"*";
			}
		}
		os<<*vi<<"-> ";
		for (auto ei=es.first; ei!=es.second; ei++){
			os<<(*ei).m_target <<"("<<g[(*ei)].probability<<")";
		}
		os<<"\t(x="<<g[*vi].endPose.p.x<<", y= "<<g[*vi].endPose.p.y<<", theta= "<<g[*vi].endPose.q.GetAngle()<<")\n";
	}
    std::cout<<os.str();
}


template <class T>
void graph_file(const int &it, const T &g, const Disturbance &goal, std::vector<vertexDescriptor>plan, const vertexDescriptor &c)
{
    char fileName[50];
	sprintf(fileName, "/tmp/graph%04i.txt", it);
	FILE * f=fopen(fileName, "w");
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		auto es=boost::out_edges(*vi, g);
		if (*vi==c){
			fprintf(f,"!");
		}
		for (vertexDescriptor vp:plan){
			if (*vi==vp){
				fprintf(f,"*");
			}
		}
		fprintf(f,"%i -> ", *vi);
		for (auto ei=es.first; ei!=es.second; ei++){
			fprintf(f, "%i (%f) ", (*ei).m_target, g[(*ei)].probability);
		}
		fprintf(f, "\t(x=%.3f, y= %.3f, theta= %.3f)\n", g[*vi].endPose.p.x, g[*vi].endPose.p.y, g[*vi].endPose.q.GetAngle());
	}
	fclose(f);
}
}
#endif