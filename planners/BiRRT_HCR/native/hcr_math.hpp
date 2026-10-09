// CommonParking implementation of Banzhaf et al., IV 2018, equations (7)-(29).
#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include "steering_functions/steering_functions.hpp"
namespace hcr {
using steering::Control;using steering::State;
struct Limits {double kappa,sigma,rho;};
inline Limits& limits(){static Limits l{1,1,1};return l;}
inline Control piece(double length,double kappa,double sigma,double rho){Control u;u.delta_s=length;u.kappa=kappa;u.sigma=sigma;u.rho=rho;return u;}
inline State advance(State q,const Control& u,double length){
 double d=u.delta_s>=0?1:-1;
 static const double x[]={.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363};
 static const double w[]={.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763};
 double phase=std::abs(u.kappa)*length+.5*std::abs(u.sigma)*length*length+std::abs(u.rho)*length*length*length/6;
 int count=std::max(1,int(std::ceil(phase/.5)));double dx=0,dy=0;
 for(int j=0;j<count;++j){double half=length/(2*count),mid=(j+.5)*length/count;
  for(int k=0;k<4;++k)for(int sign:{-1,1}){double s=mid+sign*half*x[k];double theta=q.theta+d*(u.kappa*s+.5*u.sigma*s*s+u.rho*s*s*s/6);dx+=half*w[k]*std::cos(theta);dy+=half*w[k]*std::sin(theta);}}
 q.x+=d*dx;q.y+=d*dy;q.theta+=d*(u.kappa*length+.5*u.sigma*length*length+u.rho*length*length*length/6);q.kappa=u.kappa+u.sigma*length+.5*u.rho*length*length;q.d=d;return q;
}
inline std::vector<Control> ramp(double peak,double rate,double acceleration){
 std::vector<Control> u;double a,b;
 if(peak<=rate*rate/acceleration){a=std::sqrt(peak/acceleration);b=0;}else{a=rate/acceleration;b=peak/rate-a;}
 u.push_back(piece(a,0,0,acceleration));double k=.5*acceleration*a*a,s=acceleration*a;
 if(b>1e-13){u.push_back(piece(b,k,s,0));k+=s*b;}
 u.push_back(piece(a,k,s,-acceleration));return u;
}
inline double length(const std::vector<Control>& u){double L=0;for(const auto& p:u)L+=std::abs(p.delta_s);return L;}
inline State endpoint(const std::vector<Control>& u){State q{0,0,0,0,1};for(auto p:u)q=advance(q,p,std::abs(p.delta_s));return q;}
inline std::vector<Control> reverseShape(const std::vector<Control>& u){
 std::vector<Control> z;for(auto it=u.rbegin();it!=u.rend();++it){double l=std::abs(it->delta_s);z.push_back(piece(l,it->kappa+it->sigma*l+.5*it->rho*l*l,-it->sigma-it->rho*l,it->rho));}return z;
}
inline double projection(const std::vector<Control>& u,double delta){auto q=endpoint(u);return q.x*std::cos(delta/2)+q.y*std::sin(delta/2);}
inline std::vector<Control> elementaryHalf(double parameter,double delta,int type){
 if(type==1){double a=std::cbrt(delta/(2*parameter));return {piece(a,0,0,parameter),piece(a,.5*parameter*a*a,parameter*a,-parameter)};}
 double rho=limits().rho,a=parameter/rho,b=-1.5*parameter/rho+std::sqrt(parameter*parameter/(4*rho*rho)+delta/parameter);
 if(b< -1e-10)return {};b=std::max(0.,b);double k=.5*rho*a*a;std::vector<Control> u{piece(a,0,0,rho)};
 if(b>1e-13){u.push_back(piece(b,k,parameter,0));k+=parameter*b;}u.push_back(piece(a,k,parameter,-rho));return u;
}
inline bool elementary(double distance,double delta,std::vector<Control>& half,int* selected=nullptr,int requested=0){
 if(!(distance>1e-10&&delta>1e-10))return false;
 // Equation (29): prefer the three-spiral half, then the two-spiral half.
 for(int type:{2,1}){
  if(requested!=0&&requested!=type)continue;
  double upper;
  if(type==2)upper=std::min(limits().sigma,std::cbrt(delta*limits().rho*limits().rho/2));
  else upper=std::min({limits().rho,std::sqrt(2*std::pow(limits().sigma,3)/delta),4*std::pow(limits().kappa,3)/(delta*delta)});
  double lower=upper*1e-9;auto u=elementaryHalf(upper,delta,type);if(u.empty())continue;
  double high=projection(u,delta)-distance/2,low=projection(elementaryHalf(lower,delta,type),delta)-distance/2;
  if(!std::isfinite(high)||!std::isfinite(low)||high*low>0)continue;
  for(int iteration=0;iteration<65;++iteration){double mid=(lower+upper)/2;u=elementaryHalf(mid,delta,type);double f=projection(u,delta)-distance/2;if(f*low>0){lower=mid;low=f;}else upper=mid;}
  u=elementaryHalf((lower+upper)/2,delta,type);auto q=endpoint(u);double rate=0;
  for(auto p:u)rate=std::max({rate,std::abs(p.sigma),std::abs(p.sigma+p.rho*std::abs(p.delta_s))});
  if(q.kappa>limits().kappa+1e-10||rate>limits().sigma+1e-10||std::abs(q.theta-delta/2)>1e-8||std::abs(projection(u,delta)-distance/2)>1e-8)continue;
  half=u;if(selected)*selected=type;return true;
 }
 return false;
}
inline std::vector<Control> expand(const std::vector<Control>& symbolic){
 std::vector<Control> out;
 for(auto u:symbolic){
  if(std::abs(u.delta_s)<1e-12)continue;
  if(u.rho<1e299){out.push_back(u);continue;}
  if(std::abs(u.sigma)<1e-12){u.rho=0;out.push_back(u);continue;}
  auto r=ramp(limits().kappa,limits().sigma,limits().rho);double L=length(r),d=u.delta_s>0?1:-1,t=u.sigma>0?1:-1;
  if(std::abs(std::abs(u.delta_s)-L)>1e-7)throw std::runtime_error("Unexpected symbolic HCR ramp length.");
  for(auto p:r){p.delta_s*=d;p.kappa=u.kappa+t*p.kappa;p.sigma*=t;p.rho*=t;out.push_back(p);}
 }
 return out;
}
}
