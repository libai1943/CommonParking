// Section 6 of Barraquand & Latombe (Algorithmica, 1993).
// Independent implementation: continuous reached states, finite cell indexing.
#include "arc_geometry.hpp"
#include <cstdint>
#include <queue>

namespace {
void require(bool ok,const char* message){if(!ok)mexErrMsgIdAndTxt("CommonParking:BLDijkstra","%s",message);}
const double* numeric(const mxArray* a){require(a&&mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a),"Expected real full doubles.");const double* p=mxGetPr(a);for(size_t j=0;j<mxGetNumberOfElements(a);++j)require(std::isfinite(p[j]),"Expected finite data.");return p;}
struct Node {Pose q;double length;uint32_t parent,depth,reversals;int gear,steering;};
struct Entry {uint32_t node,reversals;double length;};
struct Later {bool operator()(const Entry& a,const Entry& b)const{return a.reversals!=b.reversals?a.reversals>b.reversals:(a.length!=b.length?a.length>b.length:a.node>b.node);}};
}
#ifdef _WIN32
#define BL_EXPORT __declspec(dllexport)
#else
#define BL_EXPORT
#endif
extern "C" BL_EXPORT void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
 require(nrhs==5&&nlhs==2,"Expected five inputs and two outputs.");
 const double* s=numeric(prhs[0]);const double* g=numeric(prhs[1]);const double* b=numeric(prhs[3]);const double* p=numeric(prhs[4]);
 require(mxGetNumberOfElements(prhs[0])==3&&mxGetNumberOfElements(prhs[1])==3&&mxGetNumberOfElements(prhs[3])==4&&mxGetNumberOfElements(prhs[4])==10,"Wrong dimensions.");
 require(b[0]<b[2]&&b[1]<b[3]&&p[0]>0&&p[1]>=0&&p[2]>0&&p[3]>0&&p[4]>0&&p[4]<pi/2&&p[5]>=2&&p[5]<=9&&p[5]==std::floor(p[5])&&p[6]>0&&p[7]>=1&&p[7]<4e9&&p[8]>0&&p[9]>=2&&p[9]<4e9,"Invalid parameters.");
 Geometry geometry{p[0],p[1],p[2],{{b[0],b[1],b[2],b[3]}},{},{}};
 require(mxIsCell(prhs[2]),"Obstacles must be a cell array.");for(size_t j=0;j<mxGetNumberOfElements(prhs[2]);++j){const mxArray* a=mxGetCell(prhs[2],j);const double* v=numeric(a);size_t m=mxGetM(a);require(m>=3&&mxGetN(a)==2,"Expected convex polygons.");Poly poly;Box box;for(size_t i=0;i<m;++i){P point{v[i],v[i+m]};poly.push_back(point);box.add(point);}geometry.obstacles.push_back(poly);geometry.boxes.push_back(box);}
 uint64_t side=uint64_t(1)<<int(p[5]),cells=side*side*side;std::vector<uint8_t> seen((2*cells+7)/8,0);
 auto cell=[&](Pose q)->uint64_t{if(q[0]<b[0]||q[0]>=b[2]||q[1]<b[1]||q[1]>=b[3])return cells;uint64_t x=std::min(side-1,uint64_t((q[0]-b[0])/(b[2]-b[0])*side)),y=std::min(side-1,uint64_t((q[1]-b[1])/(b[3]-b[1])*side)),a=std::min(side-1,uint64_t(positive(q[2])/tau*side));return x+side*(y+side*a);};
 auto marked=[&](uint64_t id){return (seen[id/8]>>(id%8))&1;};auto mark=[&](uint64_t id){seen[id/8]|=uint8_t(1)<<(id%8);};
 Pose start{{s[0],s[1],s[2]}},goal{{g[0],g[1],g[2]}};require(cell(start)<cells&&cell(goal)<cells,"Endpoints must lie inside the search workspace.");uint64_t target=cell(goal);
 for(Pose q:{start,goal})for(const Poly& obs:geometry.obstacles)require(!intersect(body(q,p[0],p[1],p[2]),obs),"A task endpoint is in collision.");
 std::vector<Node> nodes;nodes.reserve(std::min(size_t(p[9]),size_t(1000000)));nodes.push_back({start,0,0,0,0,0,0});mark(cell(start));mark(cell(start)+cells);
 std::priority_queue<Entry,std::vector<Entry>,Later> open;open.push({0,0,0});uint32_t solution=UINT32_MAX;double expanded=0,checks=0;bool exhausted=false;
 auto begun=std::chrono::steady_clock::now();auto elapsed=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-begun).count();};
 const double kmax=std::tan(p[4])/p[3],dsTurn=p[6]*std::cos(p[4]);
 while(!open.empty()){
  if((static_cast<uint64_t>(expanded)&255)==0&&elapsed()>p[8]){exhausted=true;break;}
  uint32_t id=open.top().node;open.pop();Node current=nodes[id];++expanded;
  if(cell(current.q)==target){solution=id;break;}if(current.depth>=p[7])continue;
  for(int gear:{1,-1})for(int steer:{0,-1,1}){
   double ds=gear*(steer==0?p[6]:dsTurn),k=steer*kmax,turn=ds*k;Pose next=current.q;
   if(steer==0){next[0]+=ds*std::cos(next[2]);next[1]+=ds*std::sin(next[2]);}else{next[0]+=(std::sin(next[2]+turn)-std::sin(next[2]))/k;next[1]+=(std::cos(next[2])-std::cos(next[2]+turn))/k;}next[2]=wrap(next[2]+turn);
   uint64_t index=cell(next);if(index==cells)continue;index+=(gear==1?0:cells);if(marked(index))continue;
   ++checks;if(!geometry.free(current.q,ds,k))continue;
   if(nodes.size()>=p[9]){exhausted=true;break;}
   mark(index);uint32_t reversals=current.reversals+(current.gear!=0&&current.gear!=gear);double length=current.length+std::abs(ds);uint32_t newId=static_cast<uint32_t>(nodes.size());
   nodes.push_back({next,length,id,current.depth+1,reversals,gear,steer});open.push({newId,reversals,length});
  }
  if(exhausted)break;
 }
 std::vector<std::array<double,2>> reverse;if(solution!=UINT32_MAX)for(uint32_t id=solution;id!=0;id=nodes[id].parent){const Node& n=nodes[id];reverse.push_back({n.gear*(n.steering==0?p[6]:dsTurn),n.steering*kmax});}
 std::reverse(reverse.begin(),reverse.end());std::vector<std::array<double,2>> primitives;for(auto arc:reverse){if(!primitives.empty()&&primitives.back()[1]==arc[1]&&primitives.back()[0]*arc[0]>0)primitives.back()[0]+=arc[0];else primitives.push_back(arc);}
 plhs[0]=mxCreateDoubleMatrix(primitives.size(),2,mxREAL);double* out=mxGetPr(plhs[0]);for(size_t i=0;i<primitives.size();++i){out[i]=primitives[i][0];out[i+primitives.size()]=primitives[i][1];}
 plhs[1]=mxCreateDoubleMatrix(1,7,mxREAL);double stats[]={expanded,double(nodes.size()),checks,elapsed(),exhausted?1.:0.,solution==UINT32_MAX?-1.:double(nodes[solution].reversals),solution==UINT32_MAX?0.:nodes[solution].length};std::copy(stats,stats+7,mxGetPr(plhs[1]));
}
