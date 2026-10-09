// CommonParking: deterministic 2-D convex GJK distance with simplex reduction.
#pragma once
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
namespace cphc {
struct V {double x,y;};
inline V operator+(V a,V b){return {a.x+b.x,a.y+b.y};}
inline V operator-(V a,V b){return {a.x-b.x,a.y-b.y};}
inline V operator*(V a,double b){return {a.x*b,a.y*b};}
inline double dot(V a,V b){return a.x*b.x+a.y*b.y;}
inline double cross(V a,V b){return a.x*b.y-a.y*b.x;}
inline V support(const std::vector<V>& p,V d){
    V best=p[0];double value=dot(best,d);
    for(V v:p)if(dot(v,d)>value){best=v;value=dot(v,d);}return best;
}
inline V segment(V a,V b,double& t){
    V ab=b-a;double den=dot(ab,ab);t=den>1e-28?std::max(0.0,std::min(1.0,-dot(a,ab)/den)):0;return a+ab*t;
}
inline V reduce(std::vector<V>& simplex){
    if(simplex.size()==1)return simplex[0];
    if(simplex.size()==2){double t;V q=segment(simplex[0],simplex[1],t);
        if(t<=0)simplex={simplex[0]};else if(t>=1)simplex={simplex[1]};return q;}
    V a=simplex[0],b=simplex[1],c=simplex[2];
    double orientation=cross(b-a,c-a),ab=cross(b-a,a*-1),bc=cross(c-b,b*-1),ca=cross(a-c,c*-1);
    if(std::abs(orientation)>1e-24&&((ab>=0&&bc>=0&&ca>=0)||(ab<=0&&bc<=0&&ca<=0)))return {0,0};
    std::array<V,3> p={a,b,c};V best={0,0};double norm=INFINITY,bestT=0;int edge=0;
    for(int i=0;i<3;++i){double t;V q=segment(p[i],p[(i+1)%3],t);if(dot(q,q)<norm){norm=dot(q,q);best=q;edge=i;bestT=t;}}
    if(bestT<=0)simplex={p[edge]};else if(bestT>=1)simplex={p[(edge+1)%3]};else simplex={p[edge],p[(edge+1)%3]};return best;
}
inline double distance(const std::vector<V>& a,const std::vector<V>& b){
    auto minkowski=[&](V d){return support(a,d)-support(b,d*-1);};
    std::vector<V> simplex={minkowski({1,0})};V q=simplex[0];double lower=0;
    for(int i=0;i<100;++i){
        double n=dot(q,q);if(n<=1e-24)return 0;
        V w=minkowski(q*-1);
        // q is an upper bound and the support plane gives a lower bound.
        lower=std::max(lower,dot(q,w)/std::sqrt(n));
        // Return the supporting-plane LOWER bound for swept-body certificates.
        if(n-dot(q,w)<=1e-13*std::max(1.0,n))return lower;
        bool duplicate=false;for(V p:simplex)if(dot(w-p,w-p)<=1e-26)duplicate=true;
        if(duplicate)return lower;
        simplex.push_back(w);q=reduce(simplex);
    }
    return lower;
}
}
