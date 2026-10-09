// Own implementation of Model 1 (TAC 2021) and the linear graph update
// Algorithm 3 (Consolini, Laurini & Locatelli, COAP 2019 / arXiv 1809.01970).
#include "arc_geometry.hpp"
#include <cstdint>
#include <numeric>
#include <stdexcept>

namespace {
using Clock=std::chrono::steady_clock;
double seconds(Clock::time_point t){return std::chrono::duration<double>(Clock::now()-t).count();}
struct Interp {std::array<int,4> id{{-1,-1,-1,-1}};std::array<double,4> w{{0,0,0,0}};bool inside=false;};
struct Grid {
 double xmin,ymin,h,da;int nx,ny,na,n;
 int index(int x,int y,int a)const{return x+nx*(y+ny*a);}
 Pose pose(int i)const{int x=i%nx;i/=nx;int y=i%ny;int a=i/ny;return {{xmin+x*h,ymin+y*h,a*da}};}
 Interp interpolate(Pose q)const{
  Interp z;double x=(q[0]-xmin)/h,y=(q[1]-ymin)/h,a=positive(q[2])/da;
  if(x< -1e-10||y< -1e-10||x>nx-1+1e-10||y>ny-1+1e-10)return z;
  x=std::max(0.,std::min(double(nx-1),x));y=std::max(0.,std::min(double(ny-1),y));
  int ix=std::min(nx-2,int(std::floor(x))),iy=std::min(ny-2,int(std::floor(y))),ia=int(std::floor(a))%na;
  std::array<double,3> f{{x-ix,y-iy,a-std::floor(a)}};std::array<int,3> order{{0,1,2}};
  std::stable_sort(order.begin(),order.end(),[&](int i,int j){return f[i]>f[j];});
  std::array<int,3> p{{ix,iy,ia}};z.id[0]=index(p[0],p[1],p[2]);z.w[0]=1-f[order[0]];
  for(int j=0;j<3;++j){++p[order[j]];p[2]%=na;z.id[j+1]=index(p[0],p[1],p[2]);z.w[j+1]=f[order[j]]-(j<2?f[order[j+1]]:0);}
  z.inside=true;return z;
 }
 bool terminal(Pose q)const{return std::abs(q[0])<=h/2+1e-10&&std::abs(q[1])<=h/2+1e-10&&std::abs(wrap(q[2]))<=da/2+1e-10;}
};
Pose step(Pose q,double gear,double omega,double dt){
 if(std::abs(omega)<1e-14){q[0]+=gear*dt*std::cos(q[2]);q[1]+=gear*dt*std::sin(q[2]);}
 else{double next=q[2]+omega*dt;q[0]+=gear/omega*(std::sin(next)-std::sin(q[2]));q[1]+=gear/omega*(std::cos(q[2])-std::cos(next));}
 q[2]+=omega*dt;return q;
}
struct Row {std::array<int,4> id{{-1,-1,-1,-1}};std::array<double,4> w{{0,0,0,0}};double b=0,eta=0;};
struct Edge{int row;double weight;};
struct Heap {
 std::vector<int> a,pos;const std::vector<double>& key;
 Heap(int n,const std::vector<double>& k):pos(n,-1),key(k){}
 bool higher(int i,int j)const{return key[i]>key[j]||(key[i]==key[j]&&i<j);}
 void swap(int i,int j){std::swap(a[i],a[j]);pos[a[i]]=i;pos[a[j]]=j;}
 void increase(int v){int i=pos[v];if(i<0){i=int(a.size());a.push_back(v);pos[v]=i;}while(i>0){int p=(i-1)/2;if(!higher(a[i],a[p]))break;swap(i,p);i=p;}}
 int pop(){int v=a[0];pos[v]=-1;if(a.size()==1){a.pop_back();return v;}a[0]=a.back();a.pop_back();pos[a[0]]=0;int i=0;for(;;){int j=2*i+1;if(j>=int(a.size()))break;if(j+1<int(a.size())&&higher(a[j+1],a[j]))++j;if(!higher(a[j],a[i]))break;swap(i,j);i=j;}return v;}
};
bool occupied(Pose q,const Geometry& geometry){return !geometry.free(q,0,0);}
double interpolateValue(Pose q,int mode,const Grid& g,const std::vector<double>& value,double exterior){
 Interp in=g.interpolate(q);if(!in.inside)return exterior;double v=0;for(int j=0;j<4;++j)v+=in.w[j]*value[in.id[j]+mode*g.n];return v;
}
}

#ifdef _WIN32
#define CP_MEX_EXPORT __declspec(dllexport)
#else
#define CP_MEX_EXPORT
#endif
extern "C" CP_MEX_EXPORT void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
 if(nrhs!=4||nlhs!=3)mexErrMsgIdAndTxt("CommonParking:GraphBellmanInput","Expected start, polygon cell, bounds, parameters; three outputs.");
 try{
 const double* startData=mxGetPr(prhs[0]);const double* b=mxGetPr(prhs[2]);const double* p=mxGetPr(prhs[3]);
 Pose start{{startData[0],startData[1],startData[2]}};double h=p[4],dt=p[7],beta=p[8],penalty=p[9],tol=p[10],limit=p[11];int headings=int(p[5]),yawCount=int(p[6]),maximumSteps=int(p[12]);
 if(!(h>0&&dt>0&&beta>0&&beta<1&&tol>0&&headings>=4&&yawCount>=3&&yawCount%2==1&&limit>0))throw std::runtime_error("Invalid numerical settings.");
 Grid grid{b[0],b[1],h,tau/headings,int(std::round((b[2]-b[0])/h))+1,int(std::round((b[3]-b[1])/h))+1,headings,0};grid.n=grid.nx*grid.ny*grid.na;
 if(grid.nx<2||grid.ny<2||grid.n<=0||grid.n>3000000)throw std::runtime_error("Grid dimensions exceed the supported finite budget.");
 int N=2*grid.n,actions=2*yawCount;double obstacleCost=1/(1-beta),upper=obstacleCost/(1-beta);
 Geometry geometry;geometry.front=p[0];geometry.rear=p[1];geometry.half=p[2];geometry.bounds={{b[0],b[1],b[2],b[3]}};
 for(mwIndex j=0;j<mxGetNumberOfElements(prhs[1]);++j){const mxArray* cell=mxGetCell(prhs[1],j);int n=int(mxGetM(cell));const double* v=mxGetPr(cell);Poly poly;Box box;for(int i=0;i<n;++i){P q{v[i],v[i+n]};poly.push_back(q);box.add(q);}geometry.obstacles.push_back(poly);geometry.boxes.push_back(box);}
 Clock::time_point begin=Clock::now();std::vector<double> stage(grid.n),value(N,upper),best(N),error(N);std::vector<char> terminal(grid.n);
 for(int i=0;i<grid.n;++i){Pose q=grid.pose(i);stage[i]=occupied(q,geometry)?obstacleCost:1;terminal[i]=grid.terminal(q);if(terminal[i])value[i]=value[i+grid.n]=0;}
 std::vector<Row> rows(size_t(N)*actions);std::vector<size_t> offsets(N+1,0);
 for(int mode=0;mode<2;++mode)for(int i=0;i<grid.n;++i){int owner=i+mode*grid.n;Pose q=grid.pose(i);double gear=mode==0?1:-1;
  for(int control=0;control<yawCount;++control){double omega=p[3]*(-1+2.*control/(yawCount-1));Interp in=grid.interpolate(step(q,gear,omega,dt));
   for(int change=0;change<2;++change){Row& row=rows[size_t(owner)*actions+2*control+change];if(terminal[i])continue;
    row.b=stage[i]+change*penalty;int nextMode=change?1-mode:mode;
    if(!in.inside){row.b+=beta*upper;continue;}
    double self=0;for(int j=0;j<4;++j){row.id[j]=in.id[j]+nextMode*grid.n;row.w[j]=beta*in.w[j];if(row.id[j]==owner){self+=row.w[j];row.w[j]=0;row.id[j]=-1;}}
    double denominator=1-self;row.b/=denominator;for(int j=0;j<4;++j){row.w[j]/=denominator;if(row.w[j]>1e-16)++offsets[row.id[j]+1];else{row.id[j]=-1;row.w[j]=0;}}
   }
  }
 }
 std::partial_sum(offsets.begin(),offsets.end(),offsets.begin());std::vector<Edge> reverse(offsets.back());std::vector<size_t> cursor=offsets;
 for(size_t r=0;r<rows.size();++r){Row& row=rows[r];row.eta=row.b;for(int j=0;j<4;++j)if(row.id[j]>=0){row.eta+=row.w[j]*value[row.id[j]];reverse[cursor[row.id[j]]++]={int(r),row.w[j]};}}
 Heap queue(N,error);for(int i=0;i<N;++i){best[i]=std::numeric_limits<double>::infinity();for(int a=0;a<actions;++a)best[i]=std::min(best[i],rows[size_t(i)*actions+a].eta);error[i]=value[i]-best[i];if(error[i]>tol)queue.increase(i);}
 double buildTime=seconds(begin);Clock::time_point solveStart=Clock::now();uint64_t updates=0;bool timedOut=false;
 while(!queue.a.empty()){
  int i=queue.pop();double delta=error[i];value[i]-=delta;error[i]=0;++updates;
  for(size_t e=offsets[i];e<offsets[i+1];++e){const Edge& link=reverse[e];Row& row=rows[link.row];row.eta-=link.weight*delta;int owner=link.row/actions;
   if(row.eta<best[owner]){best[owner]=row.eta;error[owner]=value[owner]-best[owner];if(error[owner]>tol)queue.increase(owner);}
  }
  if(updates%65536==0&&seconds(solveStart)>limit){timedOut=true;break;}
 }
 double solveTime=seconds(solveStart),residual=0;
 // Recompute the fixed-point residual independently of incrementally cached eta.
 for(int i=0;i<N;++i){double minimum=std::numeric_limits<double>::infinity();for(int a=0;a<actions;++a){const Row& row=rows[size_t(i)*actions+a];double v=row.b;for(int j=0;j<4;++j)if(row.id[j]>=0)v+=row.w[j]*value[row.id[j]];minimum=std::min(minimum,v);}residual=std::max(residual,std::abs(value[i]-minimum));}
 int code=timedOut?-1:(residual>tol*1.1?-2:0),policySteps=0;std::vector<std::array<double,2>> arcs;Pose q=start;int mode=0;
 double startValue=interpolateValue(q,mode,grid,value,upper);
 if(code==0){
  for(;policySteps<maximumSteps;++policySteps){if(grid.terminal(q)){code=1;break;}double minimum=std::numeric_limits<double>::infinity(),omegaBest=0;int nextMode=mode;Pose next{};double gear=mode==0?1:-1,cost=occupied(q,geometry)?obstacleCost:1;
   // Dynamics use the current mode; the selected symbol changes the NEXT mode.
   for(int control=0;control<yawCount;++control){double omega=p[3]*(-1+2.*control/(yawCount-1));Pose candidate=step(q,gear,omega,dt);
    for(int change=0;change<2;++change){int nextGear=change?1-mode:mode;double candidateCost=cost+change*penalty+beta*interpolateValue(candidate,nextGear,grid,value,upper);
     if(candidateCost<minimum-1e-12){minimum=candidateCost;omegaBest=omega;nextMode=nextGear;next=candidate;}
    }
   }
   if(!grid.interpolate(next).inside){code=-3;break;}double ds=gear*dt,k=omegaBest/gear;
   if(!arcs.empty()&&arcs.back()[0]*ds>0&&std::abs(arcs.back()[1]-k)<1e-12)arcs.back()[0]+=ds;else arcs.push_back({{ds,k}});
   q=next;mode=nextMode;
  }
  if(code==0)code=grid.terminal(q)?1:-4;
 }
 plhs[0]=mxCreateDoubleMatrix(arcs.size(),2,mxREAL);double* out=mxGetPr(plhs[0]);for(size_t i=0;i<arcs.size();++i){out[i]=arcs[i][0];out[i+arcs.size()]=arcs[i][1];}
 plhs[1]=mxCreateDoubleMatrix(1,8,mxREAL);double stats[]={double(code),double(N),double(updates),residual,buildTime,solveTime,double(policySteps),startValue};std::copy(stats,stats+8,mxGetPr(plhs[1]));
 mwSize dimensions[]={mwSize(grid.nx),mwSize(grid.ny),mwSize(grid.na),2};plhs[2]=mxCreateNumericArray(4,dimensions,mxDOUBLE_CLASS,mxREAL);std::copy(value.begin(),value.end(),mxGetPr(plhs[2]));
 }catch(const std::exception& e){mexErrMsgIdAndTxt("CommonParking:GraphBellman", "%s",e.what());}
}
