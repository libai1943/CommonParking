#pragma once
// Exact convex-body arc geometry, shared mathematical construction with DPGrid.
// Own source; no external code. This copy keeps the planner self-contained.
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
}
