// Independent six-word Dubins construction in normalized coordinates.
#pragma once
#include <cmath>
#include <algorithm>
#include <limits>
namespace dg {
constexpr double PI=3.1415926535897932384626433832795;
inline double mod(double x){x=std::fmod(x,2*PI);return x<0?x+2*PI:x;}
struct Pose{double x,y,t;};
struct Curve{double s[3]={0,0,0},k[3]={0,0,0},length=std::numeric_limits<double>::infinity();int word=-1;};
inline Pose advance(Pose q,double s,double k){
 if(std::abs(k)<1e-14){q.x+=s*std::cos(q.t);q.y+=s*std::sin(q.t);}
 else {double t=q.t+s*k;q.x+=(std::sin(t)-std::sin(q.t))/k;q.y+=(std::cos(q.t)-std::cos(t))/k;q.t=t;}return q;
}
inline Curve dubins(Pose a,Pose b,double kmax,int gear){
 Curve best;double dx=b.x-a.x,dy=b.y-a.y,d=std::hypot(dx,dy)*kmax,line=std::atan2(dy,dx);
 double alpha=mod(a.t+(gear<0?PI:0)-line),beta=mod(b.t+(gear<0?PI:0)-line);
 double sa=std::sin(alpha),sb=std::sin(beta),ca=std::cos(alpha),cb=std::cos(beta),cab=std::cos(alpha-beta);
 auto accept=[&](double t,double p,double q,int k0,int k1,int k2,int word){
  double length=(t+p+q)/kmax;if(length<best.length){best.length=length;double v[3]={t,p,q};int k[3]={k0,k1,k2};for(int j=0;j<3;++j){best.s[j]=gear*v[j]/kmax;best.k[j]=gear*k[j]*kmax;}best.word=word;}
 };
 double p2=2+d*d-2*cab+2*d*(sa-sb),tmp,p;
 if(p2>=-1e-12){p=std::sqrt(std::max(0.,p2));tmp=std::atan2(cb-ca,d+sa-sb);accept(mod(-alpha+tmp),p,mod(beta-tmp),1,0,1,0);}
 p2=2+d*d-2*cab+2*d*(sb-sa);
 if(p2>=-1e-12){p=std::sqrt(std::max(0.,p2));tmp=std::atan2(ca-cb,d-sa+sb);accept(mod(alpha-tmp),p,mod(-beta+tmp),-1,0,-1,1);}
 p2=-2+d*d+2*cab+2*d*(sa+sb);
 if(p2>=-1e-12){p=std::sqrt(std::max(0.,p2));tmp=std::atan2(-ca-cb,d+sa+sb)-std::atan2(-2.,p);accept(mod(-alpha+tmp),p,mod(-beta+tmp),1,0,-1,2);}
 p2=d*d-2+2*cab-2*d*(sa+sb);
 if(p2>=-1e-12){p=std::sqrt(std::max(0.,p2));tmp=std::atan2(ca+cb,d-sa-sb)-std::atan2(2.,p);accept(mod(alpha-tmp),p,mod(beta-tmp),-1,0,1,3);}
 tmp=(6-d*d+2*cab+2*d*(sa-sb))/8;
 if(std::abs(tmp)<=1+1e-12){p=mod(2*PI-std::acos(std::max(-1.,std::min(1.,tmp))));double t=mod(alpha-std::atan2(ca-cb,d-sa+sb)+p/2);accept(t,p,mod(alpha-beta-t+p),-1,1,-1,4);}
 tmp=(6-d*d+2*cab+2*d*(-sa+sb))/8;
 if(std::abs(tmp)<=1+1e-12){p=mod(2*PI-std::acos(std::max(-1.,std::min(1.,tmp))));double t=mod(-alpha-std::atan2(ca-cb,d+sa-sb)+p/2);accept(t,p,mod(beta-alpha-t+p),1,-1,1,5);}
 return best;
}
}
