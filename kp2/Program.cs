using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
namespace kp2
{

    // Driver Code
    class Program
    {
        public class DijkstrasAlgorithm
        {

            public static void dik(List<Edge> mst, int o)
            {
                int[] path = new int[mst.Count()];
                bool[] visited = new bool[mst.Count()];
                for (int i = 0; i < mst.Count(); i++)
                {
                    visited[i] = false;
                }
                visited[o] = true;
                for(int i =0;i<0;i++)
                {
                    path[i] = int.MaxValue;
                }
                path[o] = 0;

                for(int i =0;i<mst.Count()-1;i++)
                {
                    
                }
            }
            private static readonly int NO_PARENT = -1;

            
             public static void dijkstra(int[,] adjacencyMatrix, int startVertex)
                
            {
               
                int nVertices = adjacencyMatrix.GetLength(0);

                int[] shortestDistances = new int[nVertices];

              
                bool[] added = new bool[nVertices];

                
                for (int vertexIndex = 0; vertexIndex < nVertices;
                                                    vertexIndex++)
                {
                    shortestDistances[vertexIndex] = int.MaxValue;
                    added[vertexIndex] = false;
                }

               
                shortestDistances[startVertex] = 0;

               
                int[] parents = new int[nVertices];

               
                parents[startVertex] = NO_PARENT;

            
                for (int i = 1; i < nVertices; i++)
                {

                    int nearestVertex = -1;
                    int shortestDistance = int.MaxValue;
                    for (int vertexIndex = 0;
                            vertexIndex < nVertices;
                            vertexIndex++)
                    {
                        if (!added[vertexIndex] &&
                            shortestDistances[vertexIndex] <
                            shortestDistance)
                        {
                            nearestVertex = vertexIndex;
                            shortestDistance = shortestDistances[vertexIndex];
                        }
                    }

                   
                    added[nearestVertex] = true;

                    for (int vertexIndex = 0;
                            vertexIndex < nVertices;
                            vertexIndex++)
                    {
                        int edgeDistance = adjacencyMatrix[nearestVertex, vertexIndex];

                        if (edgeDistance > 0
                            && ((shortestDistance + edgeDistance) <
                                shortestDistances[vertexIndex]))
                        {
                            parents[vertexIndex] = nearestVertex;
                            shortestDistances[vertexIndex] = shortestDistance +
                                                            edgeDistance;
                        }
                    }
                }
                
             /*    
         static int V = 10000;
            public static int minDistance(int[] dist,
                    bool[] sptSet)
    {
                Console.WriteLine(V);
                // Initialize min value
                int min = int.MaxValue, min_index = -1;
 
        for (int v = 0; v < V; v++)
            if (sptSet[v] == false && dist[v] <= min) {
                min = dist[v];
                min_index = v;
            }
 
        return min_index;
    }

            // A utility function to print
            // the constructed distance array
            public static void printSolution(int[] dist)
    {
        Console.Write("Vertex \t\t Distance "
                      + "from Source\n");
        for (int i = 0; i < V; i++)
            Console.Write(i + " \t\t " + dist[i] + "\n");
    }
             
 
                // Function that implements Dijkstra's
    // single source shortest path algorithm
    // for a graph represented using adjacency
    // matrix representation
    
    public static void dijkstra(int[, ] graph, int src)
    {
                Stopwatch stopWatch = new Stopwatch();
                stopWatch.Start();
                int[] dist = new int[V]; // The output array. dist[i]
        
        bool[] sptSet = new bool[V];
 
        
        for (int i = 0; i < V; i++) {
            dist[i] = int.MaxValue;
            sptSet[i] = false;
        }
 
      
        dist[src] = 0;
 
      
        for (int count = 0; count < V - 1; count++) {
           
            int u = minDistance(dist, sptSet);
                    Console.WriteLine(V);
           
            sptSet[u] = true;
 
           
            for (int v = 0; v < V; v++)
 
               
                if (!sptSet[v] && graph[u, v] != 0 && dist[u] != int.MaxValue && dist[u] + graph[u, v] < dist[v])
                    dist[v] = dist[u] + graph[u, v];
        }
                printSolution(dist);
    */
            // stopWatch.Stop();
              //  Console.WriteLine($"Time elapsed: {stopWatch.Elapsed.TotalMilliseconds}");
                 printSolution(startVertex, shortestDistances, parents);
            }

          
            private static void printSolution(int startVertex,
                                            int[] distances,
                                            int[] parents)
            {
                int nVertices = distances.Length;
                Console.Write("Vertex\t Distance\tPath");

                for (int vertexIndex = 0;
                        vertexIndex < nVertices;
                        vertexIndex++)
                {
                    if (vertexIndex != startVertex)
                    {
                        Console.Write("\n" + (startVertex +1)+ " -> ");
                        Console.Write((vertexIndex+1) + " \t\t ");
                        Console.Write(distances[vertexIndex] + "\t\t");
                        printPath(vertexIndex, parents);
                    }
                }
            }

           
            private static void printPath(int currentVertex,
                                        int[] parents)
            {

              
                if (currentVertex == NO_PARENT)
                {
                    return;
                }
                printPath(parents[currentVertex], parents);
                Console.Write((currentVertex+1) + " ");
            }


            public static void Main(String[] args)
            {



              int V =11; // Number of vertices in graph
                int E = 14; // Number of edges in graph
                Graph graph = new Graph(V, E);
             //   Random rand = new Random();
                 
            /*    for(int i =0;i<E;i++)
                {
                    graph.edge[i].dest = rand.Next(V);
                    graph.edge[i].src = rand.Next(V);
                    
                }
                for (int i = 0; i < E; i++)
                {
                    
                    graph.edge[i].weight = rand.Next(1,100);

                }
            */
                
                  graph.edge[0].src = 6;
                  graph.edge[0].dest = 0;
                  graph.edge[0].weight = 4000;
                  graph.edge[0].orient = false;


                  graph.edge[1].src = 0;
                  graph.edge[1].dest = 3;
                  graph.edge[1].weight = 1000;
                  graph.edge[1].orient = false;

                  graph.edge[2].src = 3;
                  graph.edge[2].dest = 1;
                  graph.edge[2].weight = 1000;
                  graph.edge[2].orient = false;

                  graph.edge[3].src = 3;
                  graph.edge[3].dest = 8;
                  graph.edge[3].weight = 500;
                  graph.edge[3].orient = false;

                  graph.edge[4].src = 0;
                  graph.edge[4].dest = 8;
                  graph.edge[4].weight = 1300;
                  graph.edge[4].orient = false;


                  graph.edge[5].src = 1;
                  graph.edge[5].dest = 8;
                  graph.edge[5].weight = 700;
                  graph.edge[5].orient = false;


                  graph.edge[6].src =1;
                  graph.edge[6].dest = 10;
                  graph.edge[6].weight = 600;
                  graph.edge[6].orient = false;


                  graph.edge[7].src = 10;
                  graph.edge[7].dest = 5;
                  graph.edge[7].weight = 800;
                  graph.edge[7].orient = false;


                  graph.edge[8].src = 5;
                  graph.edge[8].dest = 9;
                  graph.edge[8].weight = 700;
                  graph.edge[8].orient = false;


                  graph.edge[9].src = 9;
                  graph.edge[9].dest = 2;
                  graph.edge[9].weight = 800;
                  graph.edge[9].orient = false;


                  graph.edge[10].src = 1;
                  graph.edge[10].dest = 2;
                  graph.edge[10].weight = 500;
                  graph.edge[10].orient = false;


                  graph.edge[11].src = 2;
                  graph.edge[11].dest = 4;
                  graph.edge[11].weight = 500;
                  graph.edge[11].orient = false;


                  graph.edge[12].src = 4;
                  graph.edge[12].dest = 7;
                  graph.edge[12].weight = 1000;
                  graph.edge[12].orient = false;


                  graph.edge[13].src = 1;
                  graph.edge[13].dest = 0;
                  graph.edge[13].weight = 700;
                  graph.edge[13].orient = false;

                  
                // Function call
                //Edge[] result = graph.KruskalMST();


                List<Edge> result = graph.prim();
                //Console.WriteLine(result.Count);
                for (int i = 0; i < graph.edge.Length; i++)
                {

                   // Console.WriteLine($"{graph.edge[i].src}\t{graph.edge[i].dest}\t{graph.edge[i].weight}");
                }
                Console.WriteLine();
                for (int i = 0; i < result.Count; i++)
                {
                    
                   // Console.WriteLine($"{result[i].src}\t{result[i].dest}\t{result[i].weight}");
                }

                int[,] wGraph = new int[V, V];
                for (int i = 0; i < V; i++)
                {
                    for (int j = 0; j < V; j++)
                    {
                        for (int j1 = 0; j1 < V - 1; j1++)
                        {

                            if ((result[j1].src == i && result[j1].dest == j) || (result[j1].dest == i && result[j1].src == j))
                            {
                                wGraph[i, j] = result[j1].weight;
                            }
                        }
                    }
                }
                /*
                for (int i = 0; i < V; i++)
                {
                    for (int j = 0; j < V; j++)
                    {
                        Console.Write($"{wGraph[i, j]} \t");
                    }
                    Console.WriteLine();
                }
                */


                dijkstra(wGraph, 0);

            }

        }

    }
}

