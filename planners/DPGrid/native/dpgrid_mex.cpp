// Backward finite-grid dynamic programming with continuous circular arcs.
// Schildbach & Borrelli, IV 2016. Own implementation, no external source code.
#include "mex.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace {
constexpr double pi=3.14159265358979323846, tau=2*pi;
struct P { double x,y; P operator+(P b)const{return {x+b.x,y+b.y};} P operator-(P b)const{return {x-b.x,y-b.y};} P operator*(double b)const{return {x*b,y*b};} };
using Poly=std::vector<P>;
using Pose=std::array<double,3>;
double dot(P a,P b){return a.x*b.x+a.y*b.y;}
double cross(P a,P b){return a.x*b.y-a.y*b.x;}
double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
double positive(double a){a=std::fmod(a,tau);return a<0?a+tau:a;}
bool onSweep(double a,double t){return std::abs(a)<1e-10||positive((t>0?1:-1)*a)<=std::abs(t)+1e-10;}
P rotate(P a,double t){return {std::cos(t)*a.x-std::sin(t)*a.y,std::sin(t)*a.x+std::cos(t)*a.y};}
struct Box {
 double x0=std::numeric_limits<double>::infinity(),y0=x0,x1=-x0,y1=-x0;
 void add(P p){x0=std::min(x0,p.x);y0=std::min(y0,p.y);x1=std::max(x1,p.x);y1=std::max(y1,p.y);}
 bool disjoint(const Box& b)const{return x1<b.x0-1e-10||b.x1<x0-1e-10||y1<b.y0-1e-10||b.y1<y0-1e-10;}
};
Box arcBox(P p,P c,double turn){
 Box b; P r=p-c; b.add(p);b.add(c+rotate(r,turn));double a0=std::atan2(r.y,r.x),radius=std::sqrt(dot(r,r));
 for(int i=0;i<4;++i){double a=i*pi/2;if(onSweep(wrap(a-a0),turn))b.add(c+P{std::cos(a),std::sin(a)}*radius);}
 return b;
}
Poly body(Pose q,double front,double rear,double half){
 Poly out;for(P p:Poly{{front,half},{-rear,half},{-rear,-half},{front,-half}})out.push_back(P{q[0],q[1]}+rotate(p,q[2]));return out;
}
Poly hull(Poly p){
 std::sort(p.begin(),p.end(),[](P a,P b){return a.x<b.x||(a.x==b.x&&a.y<b.y);});
 p.erase(std::unique(p.begin(),p.end(),[](P a,P b){return a.x==b.x&&a.y==b.y;}),p.end());
 if(p.size()<3)return p;Poly h(2*p.size());size_t k=0;
 for(P a:p){while(k>=2&&cross(h[k-1]-h[k-2],a-h[k-1])<=0)--k;h[k++]=a;}
 size_t lower=k;for(int i=static_cast<int>(p.size())-2;i>=0;--i){while(k>lower&&cross(h[k-1]-h[k-2],p[i]-h[k-1])<=0)--k;h[k++]=p[i];}
 h.resize(k-1);return h;
}
bool intersect(const Poly& a,const Poly& b){
 for(const Poly* poly:{&a,&b})for(size_t i=0;i<poly->size();++i){
  P d=(*poly)[(i+1)%poly->size()]-(*poly)[i],n{-d.y,d.x};double amin=1e300,amax=-1e300,bmin=1e300,bmax=-1e300;
  for(P p:a){double v=dot(p,n);amin=std::min(amin,v);amax=std::max(amax,v);}for(P p:b){double v=dot(p,n);bmin=std::min(bmin,v);bmax=std::max(bmax,v);}
  if(amax<bmin-1e-10||bmax<amin-1e-10)return false;
 }return true;
}
bool arcSegment(P start,P centre,double turn,P a,P b){
 P r=start-centre,d=b-a,o=a-centre;double A=dot(d,d),B=2*dot(o,d),C=dot(o,o)-dot(r,r),disc=B*B-4*A*C;
 if(A<1e-24||disc < -1e-11*std::max(1.,B*B+std::abs(4*A*C)))return false;
 double root=std::sqrt(std::max(0.,disc));for(double t:{(-B-root)/(2*A),(-B+root)/(2*A)}){
  if(t< -1e-10||t>1+1e-10)continue;P v=a+d*std::max(0.,std::min(1.,t))-centre;
  if(onSweep(std::atan2(cross(r,v),dot(r,v)),turn))return true;
 }return false;
}
struct Geometry {
 double front,rear,half;std::array<double,4> bounds;std::vector<Poly> obstacles;std::vector<Box> boxes;
 bool free(Pose q,double ds,double k)const{
  Poly car=body(q,front,rear,half);P origin{q[0],q[1]};
  if(std::abs(k)<1e-12){
   P move{ds*std::cos(q[2]),ds*std::sin(q[2])};Poly points=car;for(P p:car)points.push_back(p+move);Poly swept=hull(points);Box box;for(P p:swept)box.add(p);
   for(size_t j=0;j<obstacles.size();++j)if(!box.disjoint(boxes[j])&&intersect(swept,obstacles[j]))return false;
  }else{
   P centre=origin+P{-std::sin(q[2]),std::cos(q[2])}*(1/k);double turn=ds*k;
   Box rearBox=arcBox(origin,centre,turn);
   if(rearBox.x0<bounds[0]-1e-9||rearBox.y0<bounds[1]-1e-9||rearBox.x1>bounds[2]+1e-9||rearBox.y1>bounds[3]+1e-9)return false;
   Box box;for(P p:car){Box b=arcBox(p,centre,turn);box.add({b.x0,b.y0});box.add({b.x1,b.y1});}
   for(size_t j=0;j<obstacles.size();++j){
    if(box.disjoint(boxes[j]))continue;const Poly& obs=obstacles[j];
    if(intersect(car,obs))return false;
    for(P p:car)for(size_t e=0;e<obs.size();++e)if(arcSegment(p,centre,turn,obs[e],obs[(e+1)%obs.size()]))return false;
    for(P p:obs)for(size_t e=0;e<car.size();++e)if(arcSegment(p,centre,-turn,car[e],car[(e+1)%car.size()]))return false;
   }
  }return true;
 }
};
void require(bool ok,const char* message){if(!ok)mexErrMsgIdAndTxt("CommonParking:DPGrid", "%s",message);}
const double* numeric(const mxArray* a){require(a&&mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a),"Expected full real double data.");const double* p=mxGetPr(a);for(size_t i=0;i<mxGetNumberOfElements(a);++i)require(std::isfinite(p[i]),"Data must be finite.");return p;}
size_t index(double v,size_t n){require(v>=1&&v<=n&&v==std::floor(v),"Invalid one-based node index.");return static_cast<size_t>(v)-1;}
}

#ifdef _WIN32
#define DP_EXPORT __declspec(dllexport)
#else
#define DP_EXPORT
#endif
extern "C" DP_EXPORT void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
 require(nrhs==8&&nlhs==2,"Expected eight inputs and two outputs.");
 const double* raw=numeric(prhs[0]);size_t n=mxGetM(prhs[0]);require(n>0&&mxGetN(prhs[0])==3,"Poses must be N-by-3.");
 std::vector<Pose> q(n);for(size_t i=0;i<n;++i)q[i]={raw[i],raw[i+n],raw[i+2*n]};
 const double* xy=numeric(prhs[1]);size_t ng=mxGetM(prhs[1]);require(mxGetN(prhs[1])==2&&mxIsCell(prhs[2])&&mxGetNumberOfElements(prhs[2])==ng,"Invalid XY groups.");
 std::vector<std::vector<size_t>> groups(ng);for(size_t g=0;g<ng;++g){const mxArray* a=mxGetCell(prhs[2],g);const double* ids=numeric(a);for(size_t j=0;j<mxGetNumberOfElements(a);++j){size_t id=index(ids[j],n);require(std::hypot(q[id][0]-xy[g],q[id][1]-xy[g+ng])<1e-9,"Group coordinate mismatch.");groups[g].push_back(id);}}
 const double* b=numeric(prhs[4]);require(mxGetNumberOfElements(prhs[4])==4&&b[0]<b[2]&&b[1]<b[3],"Invalid workspace bounds.");
 const double* p=numeric(prhs[5]);require(mxGetNumberOfElements(prhs[5])==7,"Expected seven search parameters.");
 require(p[0]>0&&p[1]>=0&&p[2]>0&&p[3]>0&&p[4]>0&&p[4]<pi&&p[5]>=1&&p[5]<=10000&&p[5]==std::floor(p[5])&&p[6]>0,"Invalid search parameters.");
 Geometry geometry{p[0],p[1],p[2],{{b[0],b[1],b[2],b[3]}},{},{}};
 require(mxIsCell(prhs[3]),"Obstacles must be a cell array.");for(size_t j=0;j<mxGetNumberOfElements(prhs[3]);++j){const mxArray* a=mxGetCell(prhs[3],j);const double* v=numeric(a);size_t m=mxGetM(a);require(m>=3&&mxGetN(a)==2,"Polygon must have at least three vertices.");Poly polygon;Box box;for(size_t i=0;i<m;++i){P point{v[i],v[i+m]};polygon.push_back(point);box.add(point);}geometry.obstacles.push_back(polygon);geometry.boxes.push_back(box);}
 const double* sid=numeric(prhs[6]);const double* gid=numeric(prhs[7]);require(mxGetNumberOfElements(prhs[6])==1&&mxGetNumberOfElements(prhs[7])==1,"Endpoint indices must be scalars.");size_t start=index(sid[0],n),goal=index(gid[0],n);
 for(Pose pose:q)require(pose[0]>=b[0]-1e-9&&pose[0]<=b[2]+1e-9&&pose[1]>=b[1]-1e-9&&pose[1]<=b[3]+1e-9,"A pose lies outside the search workspace.");
 const int unknown=std::numeric_limits<int>::max();std::vector<int> depth(n,unknown);std::vector<double> cost(n,std::numeric_limits<double>::infinity()),length(n),curvature(n);std::vector<size_t> next(n,n),frontier{goal};depth[goal]=0;cost[goal]=0;
 auto begun=std::chrono::steady_clock::now();auto elapsed=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-begun).count();};
 double expanded=0,candidates=0,checks=0;int completed=0;bool exhausted=false;
 for(int layer=0;layer<static_cast<int>(p[5])&&!frontier.empty()&&start!=goal;++layer){
  std::sort(frontier.begin(),frontier.end(),[&](size_t a,size_t b){return cost[a]<cost[b];});std::vector<size_t> future;
  for(size_t a:frontier){
   ++expanded;double ct=std::cos(q[a][2]),st=std::sin(q[a][2]);
   for(size_t g=0;g<ng;++g){
    if((g&127)==0&&elapsed()>p[6]){exhausted=true;break;}
    double wx=xy[g]-q[a][0],wy=xy[g+ng]-q[a][1],r2=wx*wx+wy*wy;if(r2<1e-16)continue;
    double dx=ct*wx+st*wy,dy=-st*wx+ct*wy,k=2*dy/r2;if(std::abs(k)>p[3]+1e-12)continue;
    double angle=std::abs(k)<1e-12?0:wrap(2*std::atan2(dy,dx));double heading=q[a][2]+angle;
    std::vector<size_t> targets;for(size_t id:groups[g])if(depth[id]>=layer+1&&std::abs(wrap(q[id][2]-heading))<=p[4]+1e-12)targets.push_back(id);if(targets.empty())continue;
    double ds=std::abs(k)<1e-12?dx:angle/k;std::array<double,2> alternatives{{ds,std::abs(k)<1e-12?ds:(angle-(angle>=0?tau:-tau))/k}};
    for(int alternative=0;alternative<(std::abs(k)<1e-12?1:2);++alternative){
     double lengthCandidate=alternatives[alternative],costCandidate=cost[a]+std::abs(lengthCandidate);if(std::abs(lengthCandidate)<1e-10)continue;
     bool improves=false;for(size_t id:targets)if(costCandidate<cost[id]-1e-10){improves=true;break;}if(!improves)continue;
     ++candidates;++checks;if(!geometry.free(q[a],lengthCandidate,k))continue;
     for(size_t id:targets)if(costCandidate<cost[id]-1e-10){if(depth[id]==unknown)future.push_back(id);depth[id]=layer+1;cost[id]=costCandidate;length[id]=lengthCandidate;curvature[id]=k;next[id]=a;}
    }
   }if(exhausted)break;
  }
  if(exhausted)break;completed=layer+1;if(depth[start]!=unknown)break;frontier.swap(future);
 }
 if(depth[start]==unknown&&completed>=static_cast<int>(p[5]))exhausted=true;
 std::vector<size_t> route;if(depth[start]!=unknown&&start!=goal){for(size_t current=start;current!=goal;current=next[current]){require(current<n&&route.size()<n&&next[current]<n,"Invalid predecessor chain.");route.push_back(current);}}
 plhs[0]=mxCreateDoubleMatrix(route.size(),7,mxREAL);double* out=mxGetPr(plhs[0]);for(size_t j=0;j<route.size();++j){size_t node=route[j],anchor=next[node];double row[]={q[anchor][0],q[anchor][1],q[anchor][2],length[node],curvature[node],double(node+1),double(anchor+1)};for(size_t k=0;k<7;++k)out[j+route.size()*k]=row[k];}
 plhs[1]=mxCreateDoubleMatrix(1,6,mxREAL);double stats[]={double(completed),expanded,candidates,checks,elapsed(),exhausted?1.:0.};std::copy(stats,stats+6,mxGetPr(plhs[1]));
}
