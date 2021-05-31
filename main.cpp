#include <iostream>
#include "Graph.h"


//int main(int argc, char* argv[])
int main()
{
	Graph hj(45);

	std::vector<Edge> vec_edges;

	Edge e1{ 0,5,14 };
	Edge e2{ 0,2,9 };
	Edge e3{ 0,1,7 };
	Edge e4{ 1,2,10 };
	Edge e5{ 1,3,15 };
	Edge e6{ 2,5,2 };
	Edge e7{ 2,3,11 };
	Edge e8{ 3,4,6 };
	Edge e9{ 4,5,9 };
	Edge e10{ 7,8,2 };

	vec_edges.push_back(e1);
	vec_edges.push_back(e2);
	vec_edges.push_back(e3);
	vec_edges.push_back(e4);
	vec_edges.push_back(e5);
	vec_edges.push_back(e6);
	vec_edges.push_back(e7);
	vec_edges.push_back(e8);
	vec_edges.push_back(e9);
	vec_edges.push_back(e10);

	Graph g(vec_edges);

	std::optional<std::vector<int>> path = g.Path(0, 5);
	
	Graph x = g.SpannigTree();
	
	try {

	//	g.AddEdge({ 5,8,2 });
		auto q = g.At(0, 1);
		auto y = g.Connected(0, 12);
		auto z = g(0, 4);

		std::cout << q << y << z;
	}
	catch (const std::exception& e) {
		std::cout << e.what();
	}



	std::cout << "Hello world!" << std::endl;
	return 0;
}
