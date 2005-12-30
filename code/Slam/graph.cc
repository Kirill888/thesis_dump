#ifndef DEPEND
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <strings.h>
#endif

#include "graph.h"
#include "random.h"
#include "matrix.h"
#include "matrix3.h"
#include "kalman2.h"


//DROS Dependancy
//------------------------
//#include "DROSMatrixC.h"
extern "C" double **matrix_alloc(unsigned int n, unsigned int m);
extern "C" int gauss_inv(double **A, unsigned int N, double **B);
extern "C" void matrix_eye(double **dest, unsigned int n);

#define min(a,b) ((a)>(b)?(b):(a))
#define max(a,b) ((a)>(b)?(a):(b))

void computeMiddlePoint(RobotPoseCov* out,
			const Gaussian2d &p1,
			const Gaussian2d &p2);

void compute_Obs(RobotPoseCov *obs, 
                 const RobotPoseCov *p1,
		 const RobotPoseCov *p2);

Graph::Graph(const Gaussian2d *Nodes, int n){
  nodes = NULL;
  edges = NULL;

  set(Nodes, n);
}

Graph::Graph(const Graph &other){
  n_nodes = other.n_nodes;
  n_edges = other.n_edges;
  nodes2edge = other.nodes2edge;
  edge2nodes = other.edge2nodes;

  if(other.nodes != NULL){
    int i;

    nodes = new Gaussian2d[n_nodes];

    for(i = 0; i < n_nodes; ++i){
      nodes[i] = other.nodes[i];
    }
  }

  if(other.edges != NULL){
    int i;

    edges = new Edge[n_edges];

    for(i = 0; i < n_edges; ++i){
      edges[i] = other.edges[i];
    }
  }

}

const Graph& Graph::operator=(const Graph &other){
  if(&other != this){
    if(edges != NULL) delete edges;
    if(nodes != NULL) delete nodes;

    n_nodes = other.n_nodes;
    n_edges = other.n_edges;

    nodes2edge = other.nodes2edge;
    edge2nodes = other.edge2nodes;

    if(other.nodes != NULL){
      int i;

      nodes = new Gaussian2d[n_nodes];

      for(i = 0; i < n_nodes; ++i){
	nodes[i] = other.nodes[i];
      }
    }

    if(other.edges != NULL){
      int i;

      edges = new Edge[n_edges];

      for(i = 0; i < n_edges; ++i){
	edges[i] = other.edges[i];
      }
    }
  }

  return *this;
}
     
void Graph::set(const Gaussian2d * Nodes, int n){
  destroy();
  nullAll();

  if(Nodes == NULL){
    nodes = NULL;
    edges = NULL;
    n_nodes = n_edges = 0;

    return;
  }

  int i;
  n_nodes = n;
  nodes = new Gaussian2d[n_nodes];
  for( i = 0; i < n_nodes; ++i) nodes[i] = Nodes[i];

  computeEdges();
}

void Graph::addNode(const Gaussian2d &n){
   Gaussian2d *prev = nodes;

   nodes = new Gaussian2d[++n_nodes];

   for(int i=0;i<n_nodes-1;++i) nodes[i] = prev[i];
   nodes[n_nodes-1] = n;

   
   if(prev != NULL) delete[](prev);
}

void Graph::computeEdges(){

  n_edges = n_nodes*(n_nodes-1)/2;
  int from,to,ei;

  edge2nodes.setSize(n_edges,2);
  nodes2edge.setSize(n_nodes,n_nodes);

  if(edges != NULL){
    delete edges;
  }

  edges = new Edge[n_edges];


  from = 0;
  ei = 0;

  while(from < n_nodes-1){

     for(to = from+1;to < n_nodes;++to){
        edge2nodes(ei ,0) = from;
        edge2nodes(ei ,1) = to;

	nodes2edge(from ,to) = ei;
	nodes2edge(to ,from) = ei;

	edges[ei].assign(nodes[from],nodes[to]);

	ei++;
     }

     nodes2edge(from , from) = -1;
     from++;
  }

}

void Graph::debug_dump(FILE*f)const{
  fprintf(f,
    "NumNodes:%d\n"
    "NumEdges:%d\n"
    "Nodes:\n", 
    n_nodes, n_edges
  );
  int i;

  for(i = 0; i < n_nodes; ++i){
     fprintf(f,"%d: %+5.2f %+5.2f\n",i+1,nodes[i].x,nodes[i].y);
  }

  fprintf(f,"Edges:\n");

  for(i = 0; i < n_edges; ++i){
     fprintf(f,"%02d: %d -> %d: l = %7.3f, var = %.3f\n"
             ,i+1,edge2nodes.get(i,0)+1,edge2nodes.get(i,1)+1
	     ,edges[i].length, edges[i].sigma
     );
  }

}

void Graph::matlab_dump(FILE *f,const char* name)const{
   int i;

   fprintf(f,"%%Graph %s\n\n",name);
   if(n_nodes == 0){
     fprintf(f,"%s = [];\n",name);
     return;
   }

   fprintf(f,"%s = [ ...",name);

   for(i = 0; i < n_nodes; ++i){
      fprintf(f,"\n%15f %10f"
                            ,nodes[i].x 
                            ,nodes[i].y);
   }

   fprintf(f,"];\n\n");

   for(i = 0; i < n_nodes; ++i){
      fprintf(f,"%s_p(1:2,1:2,%d) = [%f,%f;%f,%f];\n"
	      ,name, i+1 
	      ,nodes[i].cov[0][0]
	      ,nodes[i].cov[0][1]
	      ,nodes[i].cov[1][0]
	      ,nodes[i].cov[1][1]
              );
   }

   fprintf(f,"\n%s_edges = [...", name);
   for(i = 0; i < n_edges; ++i){
     fprintf(f,"\n%3d %3d %10f %10f"
              ,edge2nodes.get(i ,0)+1
              ,edge2nodes.get(i ,1)+1
	      ,edges[i].length
	      ,edges[i].sigma
	      );
   }
   fprintf(f,"];\n");

}

Graph Graph::load(FILE *f){
   char buf[1024];

   Graph g;
   Gaussian2d n;

   while(fgets(buf,sizeof(buf),f) != NULL){
      double x,y;

      if(sscanf(buf,"%lf%lf%lf%lf%lf%lf",&x,&y
                              ,&n.cov[0][0]
                              ,&n.cov[0][1]
                              ,&n.cov[1][0]
                              ,&n.cov[1][1]) == 6){
         n.x = x;
         n.y = y;
	 
	 g.addNode(n);
      }else{
         return g;
      }
   }

   return g;
}

void Edge::assign(const Gaussian2d &from, const Gaussian2d &to){
   double dx,dy,dx2,dy2;
   double r2;

   dx  = to.x - from.x;
   dy  = to.y - from.y;
   dx2 = dx*dx;
   dy2 = dy*dy;
   r2  = dx2 + dy2;

   double sxx = from.cov[0][0] + to.cov[0][0];
   double syy = from.cov[1][1] + to.cov[1][1];
   double sxy = from.cov[1][0] + to.cov[1][0];

   sigma2 = (dx2*sxx + dy2*syy + 2*dx*dy*sxy)/r2;

   sigma = sqrt(sigma2);
   length = sqrt(r2);

   //   angle = atan2(dy,dx);
   //   sigma_a2 = (dy2*sxx + dx2*syy - 2*dx*dy*sxy)/(r2*r2);

   computeMiddlePoint(&midPoint1, from, to);
   computeMiddlePoint(&midPoint2, to,   from);

}

double Edge::match(const Edge &e2)const{
  double dd = length - e2.length;
  double res = 0.5*dd*dd/(sigma2 + e2.sigma2);

  return res;
}


//--------------------------------------------------------------
// GraphMatcher
//-------------------------------------------------------------
GraphMatcher::GraphMatcher():M1(300),M2(300),m1(300), m2(300)
                            , md2(NULL), mapping_(NULL){
  MD2_R_THRESHOLD    = 9;
  MD2_ODO_THRESHOLD  = 16;
}

GraphMatcher::GraphMatcher(const GraphMatcher& o):M1(300),M2(300)
                            ,m1(300), m2(300)
                            ,md2(NULL), mapping_(NULL){
  n_match = o.n_match;
  N_NODES1 = o.N_NODES1;

  if(N_NODES1 > 0){
    mapping_ = new int[N_NODES1];
    memcpy(mapping_, o.mapping_, N_NODES1*sizeof(int));
  }

  MD2_R_THRESHOLD    = o.MD2_R_THRESHOLD;
  MD2_ODO_THRESHOLD  = o.MD2_ODO_THRESHOLD;
}

void GraphMatcher::match(const Graph &G1, const Graph &G2){
  setup(G1,G2);

  //Do matching
  permut(0.0,0);

  //Clean up
  cleanup();
}

void GraphMatcher::match(const Graph &G1, 
                         const Graph &G2, 
                         const RobotPoseCov &Prior){
  setup(G1,G2);

  usePrior = true;
  prior = &Prior;

  //Do matching
  permut(0.0,0);

  cleanup();
}


void GraphMatcher::setup(const Graph &G1, const Graph &G2){
  g1 = &G1; g2 = &G2;

  compute_md2();

  int maxNodes = max(g1->numNodes(), g2->numNodes());
  init_p_node(maxNodes);

  N_NODES1 = g1->numNodes();

  M1.reset(); M2.reset();
  m1.reset(); m2.reset();

  int i;

  for(i = 0; i < N_NODES1; ++i){    m1.add(i);  }

  for(i = 0; i < g2->numNodes(); ++i){    m2.add(i);  }

  pMax = p_node[3] - 3*9;

  DESTROY_ARRAY(mapping_);
  mapping_ = new int[N_NODES1];
  memset(mapping_, -1, sizeof(int)*N_NODES1);
  n_match = 0;

}

void GraphMatcher::cleanup(){
  DESTROY_ARRAY2(md2, g1->numEdges());
  g1 = g2 = NULL;
  DESTROY_ARRAY(p_node);
}

bool GraphMatcher::checkCompatibility(const RobotPoseCov *a, 
                                      const RobotPoseCov *b){
  RobotPoseCov c;

  c.set(b->x,b->y,b->rot);

  matrix3_add(c.cov,a->cov,b->cov);

  double md2 = c.mahalanobis2(*a);

  return md2 < MD2_ODO_THRESHOLD;
}

double GraphMatcher::compute_state(bool *ok){  
  int i;
  double w = 0.0;
  *ok = true;

  if(M1.nElements() < 2) return w;

  int last = M1.nElements() - 1;

  int to1 = M1[last];
  int to2 = M2[last];

  for(i = 0; i < last; ++i){
    int i1 = g1->iEdge(M1[i], to1);
    int i2 = g2->iEdge(M2[i], to2);

    double dw = md2[i1][i2];

    if(dw  > MD2_R_THRESHOLD){
      *ok = false;
      return w;
    }

    w += dw;
  }

  //Now check the prior
  bool match = true;

  if(usePrior){
    for(i = 0; match && i < last; ++i){
      int i1 = g1->iEdge(M1[i], to1);
      int i2 = g2->iEdge(M2[i], to2);

      RobotPoseCov p1,p2,obs;

      if(M1[i] > to1){
         p1 = g1->edge(i1).midPoint1;
      }else{
         p1 = g1->edge(i1).midPoint2;
      }

      if(M2[i] > to2){
         p2 = g2->edge(i2).midPoint1;
      }else{
         p2 = g2->edge(i2).midPoint2;
      }

      compute_Obs(&obs,&p1,&p2);

      match = checkCompatibility(&obs,prior);
    }

    *ok = match;
  }
  return w;
}

void GraphMatcher::storeResult(double p){
  pMax = p;

  int i;

  printf("Store Result: %f\n", p);

  memset(mapping_, -1, sizeof(int)*N_NODES1);

  for(i = 0; i < M1.nElements(); ++i){
    mapping_[M1[i]] = M2[i];

    printf(" %d<>%d",M1[i]+1,M2[i]+1);
  }

  n_match = M1.nElements();

  printf("\n");
}

void GraphMatcher::permut(double w, int lvl){
  if(m1.isEmpty() || m2.isEmpty() ) return;


  double p_best = pMaxEdge - w; //The best you can achieve now

  //If it's less than the maximum, quit
  if(p_best < pMax) return;


  int e1 = m1.remove();
  M1.add(e1);

  int e2;

  int e2stop = m2.getTail();
  e2 = m2.remove();
  bool ok = true;

  while(true){
    double dw = 0;

    //Assign e1 => e2
    M2.add(e2);

    //Compute dw
    dw = compute_state(&ok);

    if(ok){      //Only consider if all the edges matched ok
      if(dw < 0){
	ABORT("Negative md2: %f\n",dw);
      }

      double p = p_node[M1.nElements()] - (w + dw);

      if(M1.nElements() > 1 && p > pMax){
	storeResult(p);
      }

      //Go into recursion
      permut(w + dw,lvl+1);
    }

    //Release e2, if it was the last -- end the loop
    m2.add(e2);
    M2.remove();

    if(e2 == e2stop) break;

    //Get next e2
    e2 = m2.remove();
  }

  //Assign e1 to nothing
  M1.remove();
  e2 = -1;

  //Go into recursion
  permut(w, lvl+1);

  //Put back e1, return
  m1.add(e1);
}

int GraphMatcher::commonElements1(int *ind)const{
  int i;
  int n = 0;

  for(i = 0; i < N_NODES1; ++i){
    if(mapping_[i] >= 0){
      ind[n] = i;
      n++;
    }
  }

  return n;
}
int GraphMatcher::commonElements2(int *ind)const{
  int i;
  int n = 0;

  for(i = 0; i < N_NODES1; ++i){
    if(mapping_[i] >= 0){
      ind[n] = mapping_[i];
      n++;
    }
  }
  return n;
}

void GraphMatcher::destroy(){
  DESTROY_ARRAY(mapping_);
}

void GraphMatcher::compute_md2(){
  int i,j;

  md2 = new double*[g1->numEdges()];

  for(i = 0; i < g1->numEdges(); ++i){
    md2[i] = new double[g2->numEdges()];

    Edge e1 = g1->edge(i);
    for(j = 0; j < g2->numEdges(); ++j){
      double w  = e1.match(g2->edge(j));

      if(w < 0){
	ABORT("Negative md2[%d][%d]: %f\n", i,j,w);
      }

      md2[i][j] = w; 
    }
  }

}
void  GraphMatcher::init_p_node(int n){
  p_node = new double[n+1];

  p_node[0] = 0;

  for(int i = 1; i <= n;  ++i){
    p_node[i] = 9*0.5*(i-1)*i;
  }

  pMaxEdge = p_node[n];

  printf("pMaxEdge: %f\n", pMaxEdge);
}

void computeMiddlePoint(RobotPoseCov* out,
			const Gaussian2d &p1,
			const Gaussian2d &p2){
  double dx,dy,r2,r4_inv;
  dx = p2.x - p1.x;
  dy = p2.y - p1.y;

  r2 = dx*dx + dy*dy;
  r4_inv = 1.0/(r2*r2);

  //Compute the middle point and the angle
  out->x = 0.5*(p1.x + p2.x);
  out->y = 0.5*(p1.y + p2.y);
  out->rot = atan2(dy,dx);

  //Compute Covariance
  // COV = cov1  0
  //        0   cov2
  //
  // cov = jf*COV*jf'
  double jf[3][4] = {{   0.5 ,        0  ,        0.5 ,        0},
		     {    0  ,       0.5 ,         0  ,       0.5},
		     {dy*r4_inv , -dx*r4_inv , -dy*r4_inv , dx*r4_inv}};
  double jf_t[4][3];
  matrix_transpose(jf_t, jf,3,4);
  double cov[4][4];
  cov[0][0] = p1.cov[0][0]; cov[0][1] = p1.cov[0][1];
  cov[1][0] = p1.cov[1][0]; cov[1][1] = p1.cov[1][1];
  cov[2][2] = p2.cov[0][0]; cov[2][3] = p2.cov[0][1];
  cov[3][2] = p2.cov[1][0]; cov[3][3] = p2.cov[1][1];

  cov[0][2] = cov[0][3] = cov[1][2] = cov[1][3] = 0;
  cov[2][0] = cov[2][1] = cov[3][0] = cov[3][1] = 0;

  double tmp[3][4];
  matrix_multiply(tmp,jf,cov,3,4,4);
  matrix_multiply(out->cov, tmp, jf_t, 3,4,3);
}


void compute_Obs(RobotPoseCov *obs, 
                 const RobotPoseCov *p1,
		 const RobotPoseCov *p2){
  double a = angleDiffRad(p1->rot, p2->rot);
  double ca,sa;
  ca = cos(a); sa = sin(a);

  double x = p2->x*ca - p2->y*sa;
  double y = p2->x*sa + p2->y*ca;

  obs->x   = p1->x - x;
  obs->y   = p1->y - y;
  obs->rot = a;

  //Compute Covariance
  double jf[3][6] = {{ 1 ,  0 ,  y , -ca ,  sa , -y},
		     { 0 ,  1 , -x , -sa , -ca ,  x},
		     { 0 ,  0 ,  1 ,  0 ,   0 ,  -1}};
  double jf_t[6][3];
  matrix_transpose(jf_t, jf, 3,6);
  double COV[6][6]; // [cov1, 0;
                    //   0  , cov2];
  memset(COV,0,36*sizeof(double));
  COV[0][0]=p1->cov[0][0]; COV[0][1]=p1->cov[0][1]; COV[0][2]=p1->cov[0][2];
  COV[1][0]=p1->cov[1][0]; COV[1][1]=p1->cov[1][1]; COV[1][2]=p1->cov[1][2];
  COV[2][0]=p1->cov[2][0]; COV[2][1]=p1->cov[2][1]; COV[2][2]=p1->cov[2][2];

  COV[3][3]=p2->cov[0][0]; COV[3][4]=p2->cov[0][1]; COV[3][5]=p2->cov[0][2];
  COV[4][3]=p2->cov[1][0]; COV[4][4]=p2->cov[1][1]; COV[4][5]=p2->cov[1][2];
  COV[5][3]=p2->cov[2][0]; COV[5][4]=p2->cov[2][1]; COV[5][5]=p2->cov[2][2];

  double aux[3][6];
  matrix_multiply(aux, jf, COV, 3,6,6);
  matrix_multiply(obs->cov, aux, jf_t, 3,6,3);

}

void computeMeanAndAngles(double *a, double *cov,
			 const Gaussian2d *g, int n){
  int two_n = 2*n;
  int i,j;

  double mean_x = 0.0;
  double mean_y = 0.0;

  //First compute mean
  for(i = 0; i < n; ++i){
    mean_x += g[i].x;
    mean_y += g[i].y;
  }

  mean_x /= n;   mean_y /= n;
  a[0] = mean_x; a[1] = mean_y;

  int jf_r = 2+n;  int jf_c = two_n;

  double jf[jf_r][jf_c];
  double jf_t[jf_c][jf_r];

  double inv_n = 1.0/n;

  for(i = 0; i < n; ++i){
    double dx,dy,r2,jx,jy;

    dx = g[i].x - mean_x;
    dy = g[i].y - mean_y;

    r2 = dx*dx + dy*dy;

    a[i+2] = atan2(dy,dx);

    //Compute Jacobian
    jx = dy/(n*r2); jy = -dx/r2*inv_n;

    int ix = i*2; int iy = ix + 1;
    jf[0][ix] = jf[1][iy] = inv_n;
    jf[0][iy] = jf[1][ix] = 0;

    for(j = 0; j < n; ++j){
      jf[i+2][2*j  ] = jx;
      jf[i+2][2*j+1] = jy;
    }

    jf[i+2][ix] = jx*(1-n);
    jf[i+2][iy] = jy*(1-n);
  }

//   printf("JF = [\n");
//   matrix_print(jf,jf_r,jf_c," %.9e");
//   printf("];\n");

  //Compute Covariance
  int sz_cov = (n+2)*(n+2);
  memset(cov,0,sz_cov*sizeof(double));

  matrix_transpose(jf_t,jf,2+n,two_n);

  double Col[jf_r][2];
  double *Col_t;

  double aux[n+2][n+2];

  for(i = 0; i < n; ++i){
    Col_t = (double*) jf_t;
    Col_t += 2*i*jf_r;

    matrix_transpose(Col,Col_t, 2, jf_r);

    matrix_multiply(Col, Col, g[i].cov, 2+n,2,2);
    matrix_multiply(aux,Col,Col_t,2+n,2,2+n);

    matrix_add(cov,cov,aux,n+2,n+2);
  }

}

void numericalAlign(RobotPose *m2in1 
                    ,const Graph* G1, const int *ind1
                    ,const Graph* G2, const int *ind2
                    ,int ncommon, const RobotPose &x0);

void allign_graphs(RobotPoseCov* g2in1,
		   const Graph &g1, int *ind1,
		   const Graph &g2, int *ind2,
		   int n, bool optimise){

  Gaussian2d points1[n];
  Gaussian2d points2[n];
  int i,j;

  for(i = 0; i < n; ++i){
    points1[i] = g1.node(ind1[i]);
    points2[i] = g2.node(ind2[i]);
  }
  int nobs = n + 2;

  double cov1[nobs][nobs];
  double a1[nobs];
  double cov2[nobs][nobs];
  double a2[nobs];
  double obs[nobs];

  computeMeanAndAngles(a1,(double*)cov1,points1,n);
  computeMeanAndAngles(a2,(double*)cov2,points2,n);

  obs[0] = a1[0] - a2[0];
  obs[1] = a1[1] - a2[1];

  for(i = 2; i < nobs; ++i){
    obs[i] = angleDiffRad(a1[i],a2[i]);
  }

  //cov = cov1  + cov2
  //M = inv(cov)
  double **cov = matrix_alloc(nobs,nobs);
  double **M   = matrix_alloc(nobs,nobs);

  matrix_add(cov[0], cov1, cov2,nobs,nobs);

  matrix_eye(M,nobs);
  int res = gauss_inv(cov,nobs,M);

  if(res < 0){
    ABORT("AlignGraphs: Failed to compute inverse!\n");
  }

  double A[3][3];
  double A_inv[3][3];
  double sxa,sya,saa;
  double b[3] = {0,0,0};
  double xyr[3];

  A[0][0] = M[0][0];
  A[1][1] = M[1][1];
  A[0][1] = A[1][0] = 0.5*(M[1][0] + M[0][1]);

  sxa = sya = saa = 0.0;

  for(i = 2; i < nobs; ++i){
    sxa += M[i][0] + M[0][i];
    sya += M[i][1] + M[1][i];

    for(j = 2; j < nobs; ++j){
      saa += M[i][j];
    }
  }

  A[0][2] = A[2][0] = 0.5*sxa;
  A[1][2] = A[2][1] = 0.5*sya;
  A[2][2] = saa;


  for(i = 0; i < nobs; ++i){
    b[0] += obs[i]*(M[0][i] + M[i][0]);
    b[1] += obs[i]*(M[1][i] + M[i][1]);

    for(j = 2; j < nobs; ++j){
      b[2] += obs[i]*(M[j][i] + M[i][j]);
    }
  }

  matrix3_inverse(A_inv,A);
  matrix3_multiply(xyr,A_inv,b);

#if 0
  printf("cov1  = [");
  matrix_print(cov1,nobs,nobs," %.9e");
  printf("];\n");
  printf("cov2  = [");
  matrix_print(cov2,nobs,nobs," %.9e");
  printf("];\n");
  printf("cov  = [");
  matrix_print(cov[0],nobs,nobs," %.9e");
  printf("];\n");

  printf("M  = [");
  matrix_print(M[0],nobs,nobs," %.9e");
  printf("];\n");
  printf("A  = [");
  matrix_print(A,3,3," %.9e");
  printf("];\n");
  printf("A_inv  = [");
  matrix_print(A_inv,3,3," %.9e");
  printf("];\n");
  printf("b = [");
  matrix_print(b,3,1," %.9e");
  printf("];\n");
#endif

  //xyr = 0.5*inv(A)*b

  xyr[0] *= 0.5;  xyr[1] *= 0.5;  xyr[2] *= 0.5;

  //mean2in1 is the pose of the mean of the map 2 in the map 1
  RobotPoseCov mean2in1(xyr[0] + a2[0], xyr[1] + a2[1], xyr[2], A_inv);

  print_rcov("xyr0", mean2in1);

  //Compute pose of the 0,0,0 point from the mean
  g2in1->set(-a2[0],-a2[1],0, ZEROS_3x3);
  g2in1->propagateMe(mean2in1);

  if(optimise){
    GraphAlign aligner(*g2in1, points1, points2, n);
    aligner.compute();
    RobotPose tmp;

    aligner.getResult(g2in1);
  }

  //Clean up
  free(cov); free(M);
}

void allign_graphs2(RobotPoseCov* g2in1,
		   const Graph &g1, int *ind1,
		   const Graph &g2, int *ind2,
		   int n){

  int n_pairs = n/2;

  if(n_pairs < 1) return;

  int i;
  RobotPoseCov p1, p2, p_obs;

  printf("obs = [ ...\n");
  for(i = 0; i < n_pairs; ++i){
    int i1 = i*2;
    int i2 = i1+1;

    computeMiddlePoint(&p1, g1.node(ind1[i1]), g1.node(ind1[i2]));
    computeMiddlePoint(&p2, g2.node(ind2[i1]), g2.node(ind2[i2]));

    compute_Obs(&p_obs, &p1, &p2);

    print_rcov(NULL,p_obs);
    if(i == 0){
      g2in1->set(p_obs);
    }else{//Perform Kalman update
      kalman_update(g2in1, &p_obs);
    }
    //    print_rcov(NULL, *g2in1);
  }

  printf("];\n");
}



int GraphAlign::compute(){
  int nfunc = SimplexMethod(pose, this);
  //  Eval(pose);
  return nfunc;
}

double GraphAlign::Eval(double *pose){
  double x = pose[0];
  double y = pose[1];
  double ca = cos(pose[2]);
  double sa = sin(pose[2]);

  int i;
  double Sw = 0;
  double w,md2;

  //  printf(" Eval : %+6.3f %+6.3f %+6.3f (deg)", x,y, pose[2]*RADTODEG);

  for(i = 0; i < nmap; ++i){
    Gaussian2d g(g2[i]);

    g.translateMe(x,y,ca,sa);

    double A[2][2];
    double A_inv[2][2];
    double b[2] = {g1[i].x - g.x, 
		   g1[i].y - g.y};

    matrix2_add(A, g1[i].cov, g.cov);
    matrix2_inverse(A_inv, A);

    md2 = matrix2_multiply2(A_inv, b);

    w = log(matrix2_det(A)) + md2;

    //printf(",%.3f",w);

    Sw += w;
  }

  //  printf(" Sw = %.9f\n", Sw);
  return Sw;
}

void numericalAlign(RobotPose *m2in1 
                    ,const Graph* G1, const int *ind1
                    ,const Graph* G2, const int *ind2
                    ,int ncommon, const RobotPose &x0){
  Gaussian2d g1[ncommon];
  Gaussian2d g2[ncommon];
  int i;

  for(i = 0; i < ncommon; ++i){
    g1[i] = G1->node(ind1[i]);
    g2[i] = G2->node(ind2[i]);
  }

  GraphAlign aligner(x0, g1,g2,ncommon);
  aligner.compute();
  aligner.getResult(m2in1);
}


