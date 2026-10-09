// CommonParking state-lattice A* and obstacle-free Dijkstra lookup generation.
// Motion primitives and costs are supplied by the MATLAB full-model OCP.
#include "mex.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <queue>
#include <stdexcept>
#include <vector>
static void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
static const double* array(const mxArray* a){
    require(a&&mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a),"Expected real full double arrays.");
    const double* p=mxGetPr(a);for(mwSize i=0;i<mxGetNumberOfElements(a);++i)require(std::isfinite(p[i]),"Non-finite graph input.");return p;
}
struct Edge{int from,to,dx,dy,id;double cost;std::vector<double> circles;};
struct Entry{double f,g;int id;bool operator<(const Entry& b)const {return f>b.f||(f==b.f&&id>b.id);}};
struct Grid{
    int loX,loY,nx,ny,H;
    int index(int x,int y,int h)const{return x-loX+nx*(y-loY+ny*h);}
    bool inside(int x,int y)const{return x>=loX&&x<loX+nx&&y>=loY&&y<loY+ny;}
    void decode(int i,int& x,int& y,int& h)const{x=i%nx+loX;i/=nx;y=i%ny+loY;h=i/ny;}
    int size()const{return nx*ny*H;}
};
static std::vector<Edge> edgesFrom(const mxArray* a,int H){
    const double* p=array(a);int n=(int)mxGetM(a);require(mxGetN(a)==5&&n>0,"Edges require [from,to,dx,dy,cost].");
    std::vector<Edge> edges;
    for(int i=0;i<n;++i){
        for(int j=0;j<4;++j)require(std::abs(p[i+j*n]-std::round(p[i+j*n]))<1e-8,"Graph indices/displacements must be integers.");
        Edge e{(int)p[i]-1,(int)p[i+n]-1,(int)p[i+2*n],(int)p[i+3*n],i,p[i+4*n],{}};
        require(e.from>=0&&e.from<H&&e.to>=0&&e.to<H&&e.cost>0,"Invalid edge.");edges.push_back(e);
    }return edges;
}
#ifdef _WIN32
#define CP_EXPORT __declspec(dllexport)
#else
#define CP_EXPORT
#endif
extern "C" CP_EXPORT void mexFunction(int nlhs,mxArray* out[],int nrhs,const mxArray* in[]){
 try{
    require(nrhs>=1&&mxIsChar(in[0]),"First input must be a mode string.");char mode[32];require(mxGetString(in[0],mode,sizeof(mode))==0,"Invalid mode.");
    if(std::strcmp(mode,"lookup")==0){
        require(nrhs==5&&nlhs==2,"lookup requires edges, heading count, half-range, exploration extent.");
        int H=(int)mxGetScalar(in[2]),R=(int)mxGetScalar(in[3]),extent=(int)mxGetScalar(in[4]);
        require(H>0&&H<=64&&R>0&&extent>R&&extent<=256,"Invalid lookup dimensions.");
        auto edges=edgesFrom(in[1],H);std::vector<std::vector<int>> outgoing(H);
        for(int i=0;i<(int)edges.size();++i)outgoing[edges[i].from].push_back(i);
        Grid grid{-extent,-extent,2*extent+1,2*extent+1,H};int W=2*R+1;mwSize dims[4]={(mwSize)W,(mwSize)W,(mwSize)H,(mwSize)H};
        out[0]=mxCreateNumericArray(4,dims,mxDOUBLE_CLASS,mxREAL);double* table=mxGetPr(out[0]);std::fill(table,table+(size_t)W*W*H*H,INFINITY);
        out[1]=mxCreateDoubleMatrix(H,2,mxREAL);double* info=mxGetPr(out[1]);
        for(int start=0;start<H;++start){
            auto clock=std::chrono::steady_clock::now();std::vector<double> g(grid.size(),INFINITY);std::vector<char> closed(grid.size(),0);std::priority_queue<Entry> queue;
            int root=grid.index(0,0,start);g[root]=0;queue.push({0,0,root});int remaining=W*W*H,expanded=0;
            while(!queue.empty()&&remaining){
                Entry node=queue.top();queue.pop();if(node.g!=g[node.id]||closed[node.id])continue;closed[node.id]=1;
                int x,y,h;grid.decode(node.id,x,y,h);++expanded;
                // Never accept a potentially truncated free-space distance.
                require(std::abs(x)<extent&&std::abs(y)<extent,"Increase lookup exploration extent; Dijkstra reached its boundary.");
                if(std::abs(x)<=R&&std::abs(y)<=R){table[(x+R)+W*((y+R)+W*(start+H*h))]=node.g;--remaining;}
                for(int i:outgoing[h]){const Edge& e=edges[i];int xx=x+e.dx,yy=y+e.dy;if(!grid.inside(xx,yy))continue;
                    int next=grid.index(xx,yy,e.to);double value=node.g+e.cost;if(value<g[next]){g[next]=value;queue.push({value,value,next});}}
            }
            require(remaining==0,"Primitive graph cannot reach all lookup configurations.");
            info[start]=expanded;info[start+H]=std::chrono::duration<double>(std::chrono::steady_clock::now()-clock).count();
        }
    }else if(std::strcmp(mode,"search")==0){
        require(nrhs==10&&nlhs==2,"search requires edges, swept circles, obstacles, start, goal, bounds, lookup, heading count, options.");
        int H=(int)mxGetScalar(in[8]);require(H>0&&H<=64,"Invalid heading count.");auto edges=edgesFrom(in[1],H);
        require(mxIsCell(in[2])&&mxGetNumberOfElements(in[2])==edges.size(),"Swept-circle cells must match edges.");
        for(size_t j=0;j<edges.size();++j){const mxArray* a=mxGetCell(in[2],j);const double* p=array(a);require(mxGetN(a)==3,"Swept circles require x,y,r.");edges[j].circles.assign(p,p+mxGetNumberOfElements(a));}
        const double* obstacles=array(in[3]);int O=(int)mxGetM(in[3]);require(mxGetN(in[3])==3,"Obstacles require x,y,r.");
        const double* start=array(in[4]);const double* goal=array(in[5]);const double* bounds=array(in[6]);
        require(mxGetNumberOfElements(in[4])==3&&mxGetNumberOfElements(in[5])==3&&mxGetNumberOfElements(in[6])==4,"Invalid search configuration dimensions.");
        Grid grid{(int)bounds[0],(int)bounds[2],(int)(bounds[1]-bounds[0]+1),(int)(bounds[3]-bounds[2]+1),H};
        require(grid.nx>0&&grid.ny>0&&grid.nx<=1000&&grid.ny<=1000,"Invalid search bounds.");
        require(grid.inside((int)start[0],(int)start[1])&&grid.inside((int)goal[0],(int)goal[1]),"Task outside graph bounds.");
        require(start[2]>=1&&start[2]<=H&&goal[2]>=1&&goal[2]<=H,"Invalid heading indices.");
        const double* lookup=array(in[7]);const mwSize* dimensions=mxGetDimensions(in[7]);
        require(mxGetNumberOfDimensions(in[7])==4&&dimensions[0]==dimensions[1]&&dimensions[2]==(mwSize)H&&dimensions[3]==(mwSize)H,"Invalid lookup shape.");
        int W=(int)dimensions[0],R=(W-1)/2;const double* options=array(in[9]);require(mxGetNumberOfElements(in[9])==3,"Options require seconds, expansion limit, clearance.");
        std::vector<std::vector<int>> outgoing(H);for(int i=0;i<(int)edges.size();++i)outgoing[edges[i].from].push_back(i);
        auto heuristic=[&](int x,int y,int h){int dx=(int)goal[0]-x,dy=(int)goal[1]-y;
            return (std::abs(dx)<=R&&std::abs(dy)<=R)?lookup[(dx+R)+W*((dy+R)+W*(h+H*((int)goal[2]-1)))]:std::hypot((double)dx,(double)dy);};
        auto valid=[&](const Edge& e,int x,int y){size_t n=e.circles.size()/3;
            for(int o=0;o<O;++o)for(size_t i=0;i<n;++i){double dx=x+e.circles[i]-obstacles[o],dy=y+e.circles[i+n]-obstacles[o+O],r=e.circles[i+2*n]+obstacles[o+2*O]+options[2];if(dx*dx+dy*dy<=r*r)return false;}return true;};
        std::vector<double> g(grid.size(),INFINITY);std::vector<int> parent(grid.size(),-1),edgeId(grid.size(),-1);std::priority_queue<Entry> queue;
        int root=grid.index((int)start[0],(int)start[1],(int)start[2]-1),target=grid.index((int)goal[0],(int)goal[1],(int)goal[2]-1);
        g[root]=0;queue.push({heuristic((int)start[0],(int)start[1],(int)start[2]-1),0,root});int expanded=0;bool solved=false;auto clock=std::chrono::steady_clock::now();
        while(!queue.empty()&&expanded<options[1]){
            if(std::chrono::duration<double>(std::chrono::steady_clock::now()-clock).count()>=options[0])break;
            Entry node=queue.top();queue.pop();if(node.g!=g[node.id])continue;if(node.id==target){solved=true;break;}++expanded;
            int x,y,h;grid.decode(node.id,x,y,h);
            for(int i:outgoing[h]){const Edge& e=edges[i];int xx=x+e.dx,yy=y+e.dy;if(!grid.inside(xx,yy))continue;
                int next=grid.index(xx,yy,e.to);double value=node.g+e.cost;if(value>=g[next]||!valid(e,x,y))continue;
                g[next]=value;parent[next]=node.id;edgeId[next]=i;queue.push({value+heuristic(xx,yy,e.to),value,next});}
        }
        std::vector<int> route;if(solved)for(int node=target;node!=root;node=parent[node]){require(parent[node]>=0,"Corrupt lattice parent chain.");route.push_back(edgeId[node]+1);}std::reverse(route.begin(),route.end());
        out[0]=mxCreateDoubleMatrix(route.size(),1,mxREAL);for(size_t i=0;i<route.size();++i)mxGetPr(out[0])[i]=route[i];
        out[1]=mxCreateDoubleMatrix(1,4,mxREAL);double* info=mxGetPr(out[1]);info[0]=solved;info[1]=expanded;info[2]=std::chrono::duration<double>(std::chrono::steady_clock::now()-clock).count();info[3]=solved?g[target]:INFINITY;
    }else throw std::runtime_error("Unknown graph mode.");
 }catch(const std::exception& e){mexErrMsgIdAndTxt("CommonParking:LatticeGraph","%s",e.what());}
}
