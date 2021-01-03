#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <string>
#include <fstream>
#include <stack>
#include <limits>


#include "graph.h"
#include "minqueue.h"

using namespace std;
const int Infinity = numeric_limits <int>::max(); // used to initialize distances to Infinity 
//----------------------------------------------------------------------------------- 
// A function to perform BFS traversal
vector<char> BFS(graph& g, char startV)
{
   vector<char>  visited;
   queue<char>   frontierQueue;
   set<char>     discoveredSet;
   frontierQueue.push(startV);
   discoveredSet.insert(startV);

   while ( !frontierQueue.empty() )
   {
      char currentV = frontierQueue.front();
      frontierQueue.pop();
      visited.push_back(currentV);
      vector<char> adjV;
      adjV = g.neighbors(currentV);
      for(size_t i = 0;i<adjV.size();i++){
         if(discoveredSet.find(adjV[i]) == discoveredSet.end() )
         {
            frontierQueue.push(adjV[i]);
            discoveredSet.insert(adjV[i]);
         }
      }
   }
  return visited;
}
//-----------------------------------------------------------------------------------
// A function to perform DFS traversal
vector<char> DFS(graph& g, char startV)
{
  vector<char>  visited;
  stack<char>   frontierStack;
  set<char>     visitedSet;

  frontierStack.push(startV);
  
  while(!frontierStack.empty())
  {
     char currentV = frontierStack.top();
      frontierStack.pop();
	  if(visitedSet.find(currentV) == visitedSet.end() ){
		  visitedSet.insert(currentV);
		  visited.push_back(currentV);
		  vector<char> adjV;
          adjV = g.neighbors(currentV);
		  
		  for (char v : adjV)
        frontierStack.push(v);
		  
	  }
   }		 
  return visited;
}
//-----------------------------------------------------------------------------------
// A function avoid redundancy in printing contents
void print(string s, const vector<char>& V){
	 cout << s << " ";
		for (char v : V)
		 cout << v << " ";
     cout << endl;
}
//-----------------------------------------------------------------------------------
// A function to find the path from the start vertex(startV) to every visited vertex(v)
void findPath(char startV, const vector<int> &predecessor, char v)
{
	char temp = v;
	stack<char> vertices;
	vector<char> V;
	while(temp!=startV)
	{
		vertices.push(temp);
		temp = (predecessor[temp-'A'])+ 'A';
	}
	if(temp == startV)
		vertices.push(temp);
	
	while(!vertices.empty())
	{
      char c = vertices.top();
	  V.push_back(c);
	  vertices.pop();
	}

	print(" via ",V);
}
//-----------------------------------------------------------------------------------
// A function to implement Dijkstra's Method of finding the shortest path 
vector<char> DijkstraShortestPath(char startV,graph& g, vector<int> &distance,vector<int> &predV) {

    minqueue<char,int> unvisitedQueue;
	vector<char> visited;
	vector<char> c = g.vertices();
	
	// initialize values in the vector with infinity
	for(size_t i=0;i<distance.size();i++)
		distance[i] = Infinity;
	// initialize values in the vector with -1 
	for(size_t j=0;j<predV.size();j++)
		predV[j] = -1;

    distance[startV-'A'] = 0;
    unvisitedQueue.pushinorder(startV,0);

   while (!unvisitedQueue.empty()) {
      // Visit vertex with minimum distance from startV
      char currentV = unvisitedQueue.minfront();
	  unvisitedQueue.minpop();
	   if(distance[currentV-'A'] == Infinity){
		   break;
	   }
	   visited.push_back(currentV);
	   vector<char>adj = g.neighbors(currentV);
      for (char p:adj) {
         int edgeWeight = g.getweight(currentV,p);
		 int alternativePathDistance = distance[currentV - 'A'] + edgeWeight;
                  
         // If shorter path from startV to adjV is found,
         // update adjV's distance and predecessor
         if (alternativePathDistance < distance[p-'A']) {
            distance[p-'A'] = alternativePathDistance;
            predV[p-'A'] = currentV - 'A';
			unvisitedQueue.pushinorder(p,distance[p-'A']);
         }
      }
   }
	  return visited;
}
//-----------------------------------------------------------------------------------
// A helper function to build the graph 
void buildGraph(string filename, graph& g)
{
  ifstream file(filename);
  char     v;

  if (!file.good())
  {
    cout << endl;
    cout << "**Error: unable to open input file '" << filename << "'." << endl;
    cout << endl;
    return;
  }

  //
  // Input vertices as single uppercase letters:  A B C ... #
  //
  file >> v;

  while (v != '#')
  {
    g.addvertex(v);

    file >> v;
  }

  //
  // Now input edges:  Src Dest Weight ... #
  //
  char src, dest;
  int  weight;

  file >> src;

  while (src != '#')
  {
    file >> dest;
    file >> weight;

    g.addedge(src, dest, weight);

    file >> src;
  }
}
	
//-----------------------------------------------------------------------------------
int main()
{
  graph  g;
  string filename;
  char   startV;
  vector<int> distance(26,Infinity);
  vector<int> predV(26,-1);
  cout << "Enter filename containing graph data> ";
  cin >> filename;

  //
  // Let's input the graph, and then output to see what we have:
  //
  buildGraph(filename, g);
  g.output();

  //
  // now ask user for a starting vertex, and run BFS as well as
  // strongly-connected algorithms:
  //
  cout << endl;
  cout << "Enter a starting vertex or #> ";
  cin >> startV;
  

  while (startV != '#')
  {
    vector<char> visited;
    //
    // BFS:
    //
	if (g.isvertex(startV))
    {
		vector<char> adj;
		adj = g.neighbors(startV);
	    cout<<"Neighbours: ";
	    for(size_t i=0;i<adj.size();i++){
			cout<< adj.at(i)<<" ";
		}
	  cout<<endl;
      visited = BFS(g, startV);
      cout << "BFS: ";
	  for (char v : visited)
        cout << v << " ";
      cout << endl;
    }
	//
    // DFS:
    //
    if (g.isvertex(startV))
    {
      visited = DFS(g, startV);
      cout << "DFS: ";
      for (char v : visited)
        cout << v << " ";
      cout << endl;

    }
	if (g.isvertex(startV))
    {
		visited = DijkstraShortestPath(startV,g,distance,predV);
			cout << "Dijkstra: " ;

		for (char v : visited)
			cout << v << " ";

		cout << endl;
		vector<char> V;
		for(char vertice : visited)
		{
			cout <<" "<< vertice << ": " << distance[vertice-'A'];
			findPath(startV, predV,vertice);
		}
	  }

    else
    {
      cout << "Not a valid vertex in graph, ignored..." << endl;
    }

    cout << endl;
    cout << "Enter a starting vertex or #> ";
    cin >> startV;

  }
	
  //
  // done:
  //
  return 0;
}