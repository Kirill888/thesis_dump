#ifndef __GRAPH_H__
#define __GRAPH_H__

#include "array2d.h"
#include "geometry.h"
#include "util.h"
#include "simplex.h"

class Edge{
  public:
    double length;
    double sigma;
    double sigma2;

/*     double angle; */
/*     double sigma_a2; */
/*     double sigma_a; */

    RobotPoseCov midPoint1; //from -> to
    RobotPoseCov midPoint2; //to   -> from  (difference in the orientation)

    void assign(const Gaussian2d &from, const Gaussian2d &to);

    double match(const Edge &e)const;
};

class Graph{
   protected:
     Edge *edges;
     Gaussian2d *nodes;

     int  n_nodes,
          n_edges;

     Array2d nodes2edge;
     Array2d edge2nodes;

   public:
     Graph(){
       nodes = NULL;
       edges = NULL;
       n_nodes = n_edges = 0;
     }

     Graph(const Graph &other);
     Graph(const Gaussian2d *Nodes, int n);

     const Graph& operator=(const Graph &other);

     ~Graph(){destroy();}

     void set(const Gaussian2d *Nodes, int n);
     void addNode(const Gaussian2d &n);
     void computeEdges();

     void debug_dump(FILE* f = stdout)const;

     void matlab_dump(FILE* f = stdout
                     ,const char* name = "g"
		     )const;

     int numNodes()const{return n_nodes;}
     const Gaussian2d& node(int i)const{return nodes[i];}
     const Gaussian2d *getNodes()const{ return nodes;}

     int numEdges()const{return n_edges;}
     const Edge& edge(int i)const{return edges[i];}

     int iEdge(int from, int to)const{return nodes2edge.get(from, to);}
     int iNodeFrom(int edge)const    {return edge2nodes.get(edge, 0);}
     int iNodeTo(int edge)const      {return edge2nodes.get(edge, 1);}

     static Graph load(FILE *f);

     void reset(){ destroy(); nullAll();}
 private:
     void destroy(){
       if(edges != NULL) delete[] edges;
       if(nodes != NULL) delete[] nodes;
       nodes2edge.destroy();
       edge2nodes.destroy();
     }

     void nullAll(){
       nodes = NULL;
       edges = NULL;
       n_nodes = n_edges = 0;
     }
};

class GraphMatcher{
 private:
  //Trasnient variables
  FILO M1;
  FILO M2;
  FILO m1;
  FIFO m2;
  double ** md2;
  double *p_node; //p_node[i] -- log(Probability) of match i nodes

  const Graph *g1;
  const Graph *g2;
  bool usePrior;
  const RobotPoseCov *prior;
  RobotPoseCov g2in1;

  double pMaxEdge;
  double pMax;


  // Stored between matches
  int *mapping_;
  int n_match;

  int N_NODES1;

  double MD2_R_THRESHOLD;
  double MD2_ODO_THRESHOLD;

 public:
  GraphMatcher();
  GraphMatcher(const GraphMatcher& o);
  ~GraphMatcher(){ destroy(); }

  void match(const Graph &G1, const Graph &G2);
  void match(const Graph &G1, const Graph &G2, const RobotPoseCov &prior);

  int commonElements1(int *ind)const;
  int commonElements2(int *ind)const;

  int numMatch()const{return n_match;}
  int mapping(int ind1)const{ return mapping_[ind1];}


 private:
  void destroy();

  void compute_md2();
  void init_p_node(int);

  void setup(const Graph& g1, const Graph& g2);
  void cleanup();

  void permut(double w, int lvl);
  double compute_state(bool *ok);
  void storeResult(double p);

  bool checkCompatibility(const RobotPoseCov*, const RobotPoseCov*);

};

class GraphAlign: public SystemInterface{
private:
  double pose[3];
  const Gaussian2d *g1;
  const Gaussian2d *g2;
  int nmap;

public:

  GraphAlign(const RobotPose& odo0, 
             const Gaussian2d* G1, 
             const Gaussian2d* G2, int n):g1(G1),g2(G2),nmap(n){

    pose[0] = odo0.x; pose[1] = odo0.y; pose[2] = odo0.rot;
  }

  //Returns number of evaluations
  int compute();

  void getResult(RobotPose *p)const{
    p->x   = pose[0];
    p->y   = pose[1];
    p->rot = pose[2];
  }

  //SystemInterface
  double Eval(double *pose);
  int GetNumVars(){ return 3;}
  double getVar(int i){ return pose[i];}
};


void computeMiddlePoint(RobotPoseCov* out,
			const Gaussian2d &p1,
			const Gaussian2d &p2);

void allign_graphs(RobotPoseCov* g2in1,
		   const Graph &g1, int *ind1,
		   const Graph &g2, int *ind2,
		   int n, bool optimise);

#endif
