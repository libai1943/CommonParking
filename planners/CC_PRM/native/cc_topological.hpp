// CommonParking: Fraichard & Scheuer (TRO 2004), IV-D and Appendices A-B.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "steering_functions/steering_functions.hpp"
#include "steering_functions/utilities/utilities.hpp"
namespace cpcc {
using steering::State;using steering::Control;
inline Control piece(double length,double kappa,double sigma){Control u;u.delta_s=length;u.kappa=kappa;u.sigma=sigma;return u;}
inline double length(const std::vector<Control>& u){double L=0;for(auto p:u)L+=std::abs(p.delta_s);return L;}
inline State advance(State q,const Control& u){
 double L=std::abs(u.delta_s),d=u.delta_s>=0?1:-1;State z=q;
 if(std::abs(u.sigma)>1e-15)steering::end_of_clothoid(q.x,q.y,q.theta,u.kappa,u.sigma,d,L,&z.x,&z.y,&z.theta,&z.kappa);
 else if(std::abs(u.kappa)>1e-15)steering::end_of_circular_arc(q.x,q.y,q.theta,u.kappa,d,L,&z.x,&z.y,&z.theta);
 else steering::end_of_straight_line(q.x,q.y,q.theta,d,L,&z.x,&z.y);
 z.theta=q.theta+d*(u.kappa*L+.5*u.sigma*L*L);z.kappa=u.kappa+u.sigma*L;z.d=d;return z;
}
inline std::vector<Control> elementary(double delta,int d,double kappa,double sigma){
 if(std::abs(delta)<1e-14)return {};
 // Eq. (23) is equivalent to choosing the shortest feasible symmetric pair.
 double h=std::max(std::sqrt(std::abs(delta)/sigma),std::abs(delta)/kappa),sharp=delta/(d*h*h);
 return {piece(d*h,0,sharp),piece(d*h,sharp*h,-sharp)};
}
inline double chord(double alpha,double kappa,double sigma){
 State q{0,0,0,0,1};for(auto p:elementary(2*alpha,1,kappa,sigma))q=advance(q,p);return std::hypot(q.x,q.y);
}
inline std::vector<Control> topological(State start,State goal,double kappa,double sigma){
 std::vector<Control> out;State q=start;
 auto append=[&](const std::vector<Control>& u){for(auto p:u){if(std::abs(p.delta_s)<1e-13)continue;out.push_back(p);q=advance(q,p);}};
 double delta=std::atan2(std::sin(goal.theta-start.theta),std::cos(goal.theta-start.theta));
 append(elementary(delta,-1,kappa,sigma));
 // IV-D2: reach the line through the GOAL perpendicular to its heading.
 // Appendix B prints 'start' here; that contradicts IV-D2 and its construction.
 double longitudinal=(goal.x-q.x)*std::cos(q.theta)+(goal.y-q.y)*std::sin(q.theta);
 append({piece(longitudinal,0,0)});
 double lateral=-(goal.x-q.x)*std::sin(q.theta)+(goal.y-q.y)*std::cos(q.theta);
 if(std::abs(lateral)>1e-12){
  double lower=0,upper=std::acos(-1.)/4-1e-10;
  for(int i=0;i<65;++i){double alpha=(lower+upper)/2,r=chord(alpha,kappa,sigma),shift=2*r*std::sin(alpha)/std::cos(2*alpha);if(shift<std::abs(lateral))lower=alpha;else upper=alpha;}
  double alpha=(lower+upper)/2;if(lateral>0)alpha=-alpha;
  double r=chord(std::abs(alpha),kappa,sigma),back=2*r*std::cos(alpha)/std::cos(2*alpha);
  append(elementary(2*alpha,1,kappa,sigma));append({piece(-back,0,0)});append(elementary(-2*alpha,1,kappa,sigma));
 }
 return out;
}
}
