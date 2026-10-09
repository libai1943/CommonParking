#include "mex.h"
#include "dubins.hpp"
#include <vector>
#include <queue>
#include <chrono>
#include <array>
#include <string>
using dg::Pose;using dg::Curve;
struct Polygon{std::vector<std::array<double,2>>p;double xmin=1e100,xmax=-1e100,ymin=1e100,ymax=-1e100;};
struct Environment{
 std::vector<Polygon>polys;double centres[3],r,lo[2],hi[2],spacing;
 bool free(Pose q)const{
  for(double centre:centres){double x=q.x+centre*std::cos(q.t),y=q.y+centre*std::sin(q.t);
   if(x-r<lo[0]||x+r>hi[0]||y-r<lo[1]||y+r>hi[1])return false;
   for(const auto &poly:polys){if(x<poly.xmin-r||x>poly.xmax+r||y<poly.ymin-r||y>poly.ymax+r)continue;bool inside=false;
    for(size_t j=0;j<poly.p.size();++j){auto a=poly.p[j],b=poly.p[(j+1)%poly.p.size()];double ex=b[0]-a[0],ey=b[1]-a[1],u=std::max(0.,std::min(1.,((x-a[0])*ex+(y-a[1])*ey)/(ex*ex+ey*ey)));
     if(std::hypot(x-a[0]-u*ex,y-a[1]-u*ey)<=r)return false;
     if((a[1]>y)!=(b[1]>y)){double cross=a[0]+(y-a[1])*ex/ey;if(x<cross)inside=!inside;}
    }if(inside)return false;
   }
  }return true;
 }
 bool edge(Pose q,const Curve &curve)const{
  for(int j=0;j<3;++j){int n=std::max(1,int(std::ceil(std::abs(curve.s[j])/spacing)));for(int i=1;i<=n;++i)if(!free(dg::advance(q,curve.s[j]*i/n,curve.k[j])))return false;q=dg::advance(q,curve.s[j],curve.k[j]);}return true;
 }
};
Pose pose(const double *p,size_t n,size_t j){return{p[j],p[j+n],p[j+2*n]};}
struct Entry{double f,g;int id;unsigned long long serial;};
struct Later{bool operator()(const Entry&a,const Entry&b)const{return a.f>b.f||(a.f==b.f&&a.serial>b.serial);}};
#ifdef _WIN32
#define DG_EXPORT __declspec(dllexport)
#else
#define DG_EXPORT
#endif
extern "C" DG_EXPORT void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
 if(nrhs<1||!mxIsChar(prhs[0]))mexErrMsgIdAndTxt("DubinsGrid:Input","First input is a command.");
 char *command=mxArrayToString(prhs[0]);std::string mode(command);mxFree(command);
 if(mode=="dubins"){
  if(nrhs!=5||nlhs!=2)mexErrMsgIdAndTxt("DubinsGrid:Input","dubins requires start, goal, kappa and gear and two outputs.");
  size_t n=mxGetM(prhs[1]);if(mxGetN(prhs[1])!=3||mxGetM(prhs[2])!=n||mxGetN(prhs[2])!=3)mexErrMsgIdAndTxt("DubinsGrid:Input","Pose arrays must be matching N by 3.");
  plhs[0]=mxCreateDoubleMatrix(n,6,mxREAL);plhs[1]=mxCreateDoubleMatrix(n,1,mxREAL);double *p=mxGetPr(plhs[0]),*l=mxGetPr(plhs[1]);
  for(size_t j=0;j<n;++j){Curve c=dg::dubins(pose(mxGetPr(prhs[1]),n,j),pose(mxGetPr(prhs[2]),n,j),mxGetScalar(prhs[3]),int(mxGetScalar(prhs[4])));for(int k=0;k<3;++k){p[j+n*k]=c.s[k];p[j+n*(k+3)]=c.k[k];}l[j]=c.length;}return;
 }
 if(mode!="search"||nrhs!=7||nlhs!=3)mexErrMsgIdAndTxt("DubinsGrid:Input","search requires poses, polygons, centres, radius, bounds, parameters and three outputs.");
 size_t n=mxGetM(prhs[1]);if(mxGetN(prhs[1])!=3||n<2||!mxIsCell(prhs[2])||mxGetNumberOfElements(prhs[3])!=3||mxGetM(prhs[5])!=2||mxGetN(prhs[5])!=2||mxGetNumberOfElements(prhs[6])!=4)mexErrMsgIdAndTxt("DubinsGrid:Input","Invalid dimensions.");
 auto begin=std::chrono::steady_clock::now();auto seconds=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();};
 const double *p=mxGetPr(prhs[1]),*par=mxGetPr(prhs[6]),*bounds=mxGetPr(prhs[5]);double kmax=par[0],maximum=par[1],budget=par[2];
 Environment env;env.r=mxGetScalar(prhs[4]);env.spacing=par[3];for(int j=0;j<3;++j)env.centres[j]=mxGetPr(prhs[3])[j];env.lo[0]=bounds[0];env.hi[0]=bounds[1];env.lo[1]=bounds[2];env.hi[1]=bounds[3];
 for(size_t i=0;i<mxGetNumberOfElements(prhs[2]);++i){const mxArray *cell=mxGetCell(prhs[2],i);if(!cell||mxGetN(cell)!=2)mexErrMsgIdAndTxt("DubinsGrid:Input","Polygon must be N by 2.");const double*v=mxGetPr(cell);size_t m=mxGetM(cell);Polygon poly;for(size_t j=0;j<m;++j){poly.p.push_back({v[j],v[j+m]});poly.xmin=std::min(poly.xmin,v[j]);poly.xmax=std::max(poly.xmax,v[j]);poly.ymin=std::min(poly.ymin,v[j+m]);poly.ymax=std::max(poly.ymax,v[j+m]);}env.polys.push_back(poly);}
 std::vector<Pose>q(n);std::vector<char>free(n),closed(n,0);size_t freeCount=0;
 for(size_t i=0;i<n;++i){q[i]=pose(p,n,i);free[i]=env.free(q[i]);freeCount+=free[i];}
 std::vector<double>dist(n,INFINITY),heuristic(n,-1);std::vector<int>parent(n,-1);std::vector<Curve>parentEdge(n);std::priority_queue<Entry,std::vector<Entry>,Later>heap;unsigned long long serial=0,candidates=0,checks=0;int expanded=0,reopened=0;bool found=false,expired=false;
 auto h=[&](int i){if(heuristic[i]<0)heuristic[i]=std::min(dg::dubins(q[i],q[1],kmax,1).length,dg::dubins(q[i],q[1],kmax,-1).length);return heuristic[i];};
 if(free[0]&&free[1]){dist[0]=0;heap.push({h(0),0,0,serial++});}
 while(!heap.empty()){
  if(seconds()>budget){expired=true;break;}Entry entry=heap.top();heap.pop();int i=entry.id;if(entry.g!=dist[i]||closed[i])continue;if(i==1){found=true;break;}closed[i]=1;++expanded;
  for(size_t j=1;j<n;++j){if(j==size_t(i)||!free[j])continue;double chord=std::hypot(q[j].x-q[i].x,q[j].y-q[i].y);if(chord>maximum||dist[i]+chord>=dist[j]-1e-10)continue;
   if((++candidates&1023)==0&&seconds()>budget){expired=true;break;}
   Curve forward=dg::dubins(q[i],q[j],kmax,1),reverse=dg::dubins(q[i],q[j],kmax,-1);Curve pair[2]={forward,reverse};if(pair[1].length<pair[0].length)std::swap(pair[0],pair[1]);
   for(const Curve &edge:pair){double value=dist[i]+edge.length;if(edge.length>maximum||value>=dist[j]-1e-10)continue;++checks;if(!env.edge(q[i],edge))continue;
    dist[j]=value;parent[j]=i;parentEdge[j]=edge;if(closed[j]){closed[j]=0;++reopened;}heap.push({value+h(int(j)),value,int(j),serial++});break;
   }
  }if(expired)break;
 }
 std::vector<int>route;if(found){int i=1;while(i!=0&&i>=0){route.push_back(i);i=parent[i];}std::reverse(route.begin(),route.end());}
 size_t m=route.size();plhs[0]=mxCreateDoubleMatrix(m*3,2,mxREAL);double*out=mxGetPr(plhs[0]);for(size_t i=0;i<m;++i){Curve c=parentEdge[route[i]];for(int j=0;j<3;++j){out[3*i+j]=c.s[j];out[3*i+j+3*m]=c.k[j];}}
 plhs[1]=mxCreateDoubleMatrix(1,9,mxREAL);double*s=mxGetPr(plhs[1]);s[0]=found;s[1]=expanded;s[2]=double(candidates);s[3]=double(checks);s[4]=seconds();s[5]=expired;s[6]=freeCount;s[7]=reopened;s[8]=found?dist[1]:INFINITY;
 plhs[2]=mxCreateDoubleMatrix(found?m+1:0,3,mxREAL);if(found){double*r=mxGetPr(plhs[2]);route.insert(route.begin(),0);for(size_t j=0;j<route.size();++j){Pose a=q[route[j]];r[j]=a.x;r[j+m+1]=a.y;r[j+2*(m+1)]=a.t;}}
}
