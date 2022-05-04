using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;

namespace kp2
{
    public class Edge : IComparable<Edge>
    {
        public int src, dest, weight;
        public Boolean orient;
        
        public int CompareTo(Edge compareEdge)
        {
            return  this.weight
                   - compareEdge.weight;
        }
    }
    public class Graph
    {
        public Edge[] edge;
        int V, E;

        public Graph(int v, int e)
        {
            V = v;
            E = e;
            edge = new Edge[E];
            for (int i = 0; i < e; ++i)
                edge[i] = new Edge();
            
        }
        public List<Edge> prim()
        {
            Stopwatch stopWatch = new Stopwatch();
            stopWatch.Start();
            List<Edge> result = new List<Edge>();
            List<double> ver = new List<double>();
            Array.Sort(edge);
            ver.Add(edge[0].src);
            ver.Add(edge[0].dest);
            result.Add(edge[0]);
            
            while(result.Count<V-1)
            {
                for (int i = 1; i < E; i++)
                {
                   // Console.WriteLine(i);

                    if ((!ver.Contains(edge[i].dest) && ver.Contains(edge[i].src))||((ver.Contains(edge[i].dest) && !ver.Contains(edge[i].src))))
                    {
                        //if (!ver.Contains(edge[i].dest))
                            ver.Add(edge[i].dest);
                        //if (!ver.Contains(edge[i].src))
                            ver.Add(edge[i].src);
                        result.Add(edge[i]);
                        // Console.WriteLine($"{i} added");
                        

                    }

                }
            }
            //  for (int g = 0; g < ver.Count; g++)
            // {
            // Console.WriteLine(ver[g]);
            //}
            stopWatch.Stop();
           // Console.WriteLine($"Time elapsed: {stopWatch.Elapsed.TotalMilliseconds}");

            return result;
        }


    }
}
