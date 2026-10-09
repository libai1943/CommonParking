// Bicchi, Casalino & Santilli (ICRA 1995), Algorithm 1. Own implementation.
#include "arc_geometry.hpp"
#include <map>
#include <queue>
struct Circle{P c;double radius;std::map<long long,int>points;};
struct PointOnCircle{double angle;int plus,minus;};
struct Node{Pose q;bool free;};
struct Edge{int to;double ds,k;};
struct QueueNode{double cost;int id;bool operator>(const QueueNode&b)const{return cost>b.cost||(cost==b.cost&&id>b.id);}};
#ifdef _WIN32
#define BT_EXPORT __declspec(dllexport)
#else
#define BT_EXPORT
#endif
extern "C" BT_EXPORT void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
 if(nrhs!=5||nlhs!=2||mxGetN(prhs[0])!=3||mxGetM(prhs[1])!=2||mxGetN(prhs[1])!=2||mxGetM(prhs[2])!=2||mxGetN(prhs[2])!=3||!mxIsCell(prhs[3])||mxGetNumberOfElements(prhs[4])!=5)mexErrMsgIdAndTxt("BicchiTangents:Input","Expected circles, endpoint circle IDs, two task poses, polygons and parameters.");
 auto begin=std::chrono::steady_clock::now();auto time=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();};
 const double *par=mxGetPr(prhs[4]);double budget=par[3],tolerance=par[4];Geometry geometry;geometry.front=par[0];geometry.rear=par[1];geometry.half=par[2];geometry.bounds={-INFINITY,-INFINITY,INFINITY,INFINITY};
 for(size_t j=0;j<mxGetNumberOfElements(prhs[3]);++j){const mxArray *a=mxGetCell(prhs[3],j);size_t n=mxGetM(a);const double*p=mxGetPr(a);Poly poly;Box box;for(size_t i=0;i<n;++i){P point{p[i],p[i+n]};poly.push_back(point);box.add(point);}geometry.obstacles.push_back(poly);geometry.boxes.push_back(box);}
 size_t nc=mxGetM(prhs[0]);const double *cc=mxGetPr(prhs[0]),*tasks=mxGetPr(prhs[2]),*endpoint=mxGetPr(prhs[1]);std::vector<Circle>circles(nc);
 for(size_t j=0;j<nc;++j){circles[j].c={cc[j],cc[j+nc]};circles[j].radius=cc[j+2*nc];}
 std::vector<Node>nodes;std::vector<std::vector<Edge>>adjacency;std::vector<PointOnCircle>points;int candidateEdges=0,freeEdges=0,expanded=0;bool expired=false;
 auto addNode=[&](Pose q){int id=int(nodes.size());nodes.push_back({q,geometry.free(q,0,0)});adjacency.emplace_back();return id;};
 for(int j=0;j<2;++j)addNode({tasks[j],tasks[j+2],tasks[j+4]});
 auto point=[&](int circle,double a){
  a=positive(a);long long key=std::llround(a/tolerance);if(std::abs(a-tau)<2*tolerance)key=0;
  auto found=circles[circle].points.find(key);if(found!=circles[circle].points.end())return found->second;
  P p=circles[circle].c+P{std::cos(a),std::sin(a)}*circles[circle].radius;int plus=addNode({p.x,p.y,a+pi/2}),minus=addNode({p.x,p.y,a-pi/2});int id=int(points.size());points.push_back({a,plus,minus});circles[circle].points[key]=id;return id;
 };
 auto edge=[&](int from,int to,double ds,double k){
  ++candidateEdges;if(!nodes[from].free||!nodes[to].free||!geometry.free(nodes[from].q,ds,k))return;
  Pose q=nodes[from].q;double turn=ds*k,xx,yy;if(std::abs(k)<1e-12){xx=q[0]+ds*std::cos(q[2]);yy=q[1]+ds*std::sin(q[2]);}else{xx=q[0]+(std::sin(q[2]+turn)-std::sin(q[2]))/k;yy=q[1]+(std::cos(q[2])-std::cos(q[2]+turn))/k;}
  const Pose&r=nodes[to].q;if(std::hypot(xx-r[0],yy-r[1])>1e-7||std::abs(wrap(q[2]+turn-r[2]))>1e-7)mexErrMsgIdAndTxt("BicchiTangents:Geometry","A constructed graph edge fails the exact endpoint check.");
  adjacency[from].push_back({to,ds,k});++freeEdges;
 };
 // Start and goal are exact graph nodes, connected to both supporting circles.
 for(int j=0;j<2;++j)for(int side=0;side<2;++side){int ci=int(endpoint[j+2*side])-1;if(ci<0||ci>=int(nc))mexErrMsgIdAndTxt("BicchiTangents:Input","Invalid endpoint circle ID.");P radial=P{tasks[j],tasks[j+2]}-circles[ci].c;int p=point(ci,std::atan2(radial.y,radial.x));int id=std::abs(wrap(nodes[points[p].plus].q[2]-tasks[j+4]))<1e-7?points[p].plus:points[p].minus;edge(j,id,0,0);edge(id,j,0,0);}
 for(size_t i=0;i<nc&&!expired;++i)for(size_t j=i+1;j<nc;++j){
  if(time()>budget){expired=true;break;}P d=circles[j].c-circles[i].c;double distance=std::sqrt(dot(d,d));if(distance<1e-10)continue;P direction=d*(1/distance),normal{-direction.y,direction.x};
  // signedRadius chooses exterior or interior common tangents. Each tangent
  // has both body orientations and both physical traversal directions.
  for(int sign:{1,-1}){double projection=(circles[i].radius-sign*circles[j].radius)/distance;if(std::abs(projection)>1+1e-12)continue;projection=std::max(-1.,std::min(1.,projection));double lateral=std::sqrt(std::max(0.,1-projection*projection));
   for(int branch:{1,-1}){if(branch==-1&&lateral<1e-12)continue;P n=direction*projection+normal*(branch*lateral);P a=circles[i].c+n*circles[i].radius,b=circles[j].c+n*(sign*circles[j].radius);P delta=b-a;double length=std::sqrt(dot(delta,delta));P tangent=length>1e-10?delta*(1/length):P{-n.y,n.x};double heading=std::atan2(tangent.y,tangent.x);
    int pi=point(int(i),std::atan2(n.y,n.x)),pj=point(int(j),std::atan2(sign*n.y,sign*n.x));
    auto oriented=[&](int p,double theta){return std::abs(wrap(nodes[points[p].plus].q[2]-theta))<1e-7?points[p].plus:points[p].minus;};
    int aForward=oriented(pi,heading),bForward=oriented(pj,heading),aBackward=oriented(pi,heading+::pi),bBackward=oriented(pj,heading+::pi);
    edge(aForward,bForward,length,0);edge(bForward,aForward,-length,0);edge(aBackward,bBackward,-length,0);edge(bBackward,aBackward,length,0);
   }
  }
 }
 // Adjacent angular arcs generate every longer arc by concatenation, with
 // identical geometry/cost and no loss of the original complete arc graph.
 for(size_t ci=0;ci<nc&&!expired;++ci){std::vector<int>order;for(auto item:circles[ci].points)order.push_back(item.second);size_t n=order.size();if(n<2)continue;
  for(size_t j=0;j<n;++j){if(time()>budget){expired=true;break;}auto a=points[order[j]],b=points[order[(j+1)%n]];double delta=positive(b.angle-a.angle),length=delta*circles[ci].radius,k=1/circles[ci].radius;
   edge(a.plus,b.plus,length,k);edge(b.plus,a.plus,-length,k);edge(a.minus,b.minus,-length,-k);edge(b.minus,a.minus,length,-k);
  }
 }
 double buildTime=time();std::vector<double>dist(nodes.size(),INFINITY);std::vector<int>parent(nodes.size(),-1);std::vector<Edge>parentEdge(nodes.size());std::priority_queue<QueueNode,std::vector<QueueNode>,std::greater<QueueNode>>heap;bool found=false;
 if(!expired&&nodes[0].free&&nodes[1].free){dist[0]=0;heap.push({0,0});}
 while(!heap.empty()){
  if(time()>budget){expired=true;break;}auto top=heap.top();heap.pop();if(top.cost!=dist[top.id])continue;if(top.id==1){found=true;break;}++expanded;
  for(auto e:adjacency[top.id]){double cost=top.cost+std::abs(e.ds);if(cost<dist[e.to]-1e-10){dist[e.to]=cost;parent[e.to]=top.id;parentEdge[e.to]=e;heap.push({cost,e.to});}}
 }
 std::vector<std::array<double,5>>route;if(found){for(int i=1;i!=0;i=parent[i]){int from=parent[i];auto q=nodes[from].q;auto e=parentEdge[i];if(std::abs(e.ds)>1e-10)route.push_back({q[0],q[1],q[2],e.ds,e.k});}std::reverse(route.begin(),route.end());}
 size_t n=route.size();plhs[0]=mxCreateDoubleMatrix(n,5,mxREAL);double*out=mxGetPr(plhs[0]);for(size_t i=0;i<n;++i)for(int j=0;j<5;++j)out[i+n*j]=route[i][j];
 plhs[1]=mxCreateDoubleMatrix(1,11,mxREAL);double*s=mxGetPr(plhs[1]);s[0]=found;s[1]=nc;s[2]=nodes.size();s[3]=0;for(auto node:nodes)s[3]+=node.free;s[4]=candidateEdges;s[5]=freeEdges;s[6]=expanded;s[7]=buildTime;s[8]=time()-buildTime;s[9]=found?dist[1]:INFINITY;s[10]=expired;
}
