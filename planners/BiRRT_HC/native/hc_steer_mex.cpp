// MATLAB gateway for the authors' HC+- steering functions (Apache-2.0).
// CommonParking additions: batch connections and exact arc-length sampling.
#include "mex.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>
#include "steering_functions/hc_cc_state_space/hcpmpm_reeds_shepp_state_space.hpp"
#include "steering_functions/utilities/utilities.hpp"
#include "gjk_distance.hpp"
using steering::State;
using steering::Control;
static void require(bool ok,const char* message) {
    if(!ok) throw std::runtime_error(message);
}
static const double* realArray(const mxArray* a) {
    require(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a),"Expected a full real double array.");
    const double* p=mxGetPr(a);
    for(mwSize i=0;i<mxGetNumberOfElements(a);++i) require(std::isfinite(p[i]),"Inputs must be finite.");
    return p;
}
static State rowState(const mxArray* a,mwSize i) {
    const double* p=realArray(a);mwSize n=mxGetM(a);
    require(mxGetN(a)==4&&n>0,"Configuration array must be N-by-4: x,y,theta,kappa.");
    i=(n==1)?0:i;require(i<n,"Incompatible batch sizes.");
    return State{p[i],p[i+n],p[i+2*n],p[i+3*n],0};
}
static State advance(State q,const Control& u,double length) {
    double d=(u.delta_s>=0)?1.0:-1.0;
    q.kappa=u.kappa;
    State z=q;
    if(std::abs(u.sigma)>1e-14)
        steering::end_of_clothoid(q.x,q.y,q.theta,u.kappa,u.sigma,d,length,&z.x,&z.y,&z.theta,&z.kappa);
    else if(std::abs(u.kappa)>1e-14)
        steering::end_of_circular_arc(q.x,q.y,q.theta,u.kappa,d,length,&z.x,&z.y,&z.theta);
    else steering::end_of_straight_line(q.x,q.y,q.theta,d,length,&z.x,&z.y);
    // Preserve absolute heading, including complete revolutions.
    z.theta=q.theta+d*(u.kappa*length+0.5*u.sigma*length*length);
    z.kappa=u.kappa+u.sigma*length;z.d=d;return z;
}
#ifdef _WIN32
#define CP_EXPORT __declspec(dllexport)
#else
#define CP_EXPORT
#endif
extern "C" CP_EXPORT void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {
  try {
    require(nrhs>=1&&mxIsChar(prhs[0]),"First input must be a mode string.");
    char mode[32];require(mxGetString(prhs[0],mode,sizeof(mode))==0,"Mode is too long.");
    if(std::strcmp(mode,"connect")==0) {
        require(nrhs==4&&nlhs>=1&&nlhs<=2,"connect requires q1, q2, [kappa_max,sigma_max]; returns controls and lengths.");
        const double* limits=realArray(prhs[3]);
        require(mxGetNumberOfElements(prhs[3])==2&&limits[0]>0&&limits[1]>0,"Invalid steering limits.");
        mwSize n1=mxGetM(prhs[1]),n2=mxGetM(prhs[2]),n=std::max(n1,n2);
        require(n1==n2||n1==1||n2==1,"Incompatible batch sizes.");
        steering::HCpmpm_Reeds_Shepp_State_Space space(limits[0],limits[1]);
        plhs[0]=mxCreateCellMatrix(n,1);
        if(nlhs>1) plhs[1]=mxCreateDoubleMatrix(n,1,mxREAL);
        for(mwSize i=0;i<n;++i) {
            State a=rowState(prhs[1],i),b=rowState(prhs[2],i);
            auto raw=space.get_controls(a,b);std::vector<Control> u;
            for(auto c:raw) if(std::abs(c.delta_s)>1e-12) u.push_back(c);
            mxArray* out=mxCreateDoubleMatrix(u.size(),3,mxREAL);double* p=mxGetPr(out);double total=0;
            for(mwSize j=0;j<u.size();++j) {
                p[j]=u[j].delta_s;p[j+u.size()]=u[j].kappa;p[j+2*u.size()]=u[j].sigma;
                total+=std::abs(u[j].delta_s);
            }
            mxSetCell(plhs[0],i,out);if(nlhs>1) mxGetPr(plhs[1])[i]=total;
        }
    } else if(std::strcmp(mode,"sample")==0) {
        require(nrhs==4&&nlhs==1,"sample requires initial [x,y,theta,kappa], controls, and arc lengths.");
        State q=rowState(prhs[1],0);require(mxGetM(prhs[1])==1,"Only one initial state is accepted.");
        const double* p=realArray(prhs[2]);mwSize n=mxGetM(prhs[2]);
        require(mxGetN(prhs[2])==3,"Controls must be N-by-3: signed_length,kappa,sharpness.");
        const double* s=realArray(prhs[3]);mwSize m=mxGetNumberOfElements(prhs[3]);
        std::vector<Control> u;double total=0;
        for(mwSize i=0;i<n;++i) {
            Control c{p[i],p[i+n],p[i+2*n]};
            if(std::abs(c.delta_s)>1e-12) {u.push_back(c);total+=std::abs(c.delta_s);}
        }
        plhs[0]=mxCreateDoubleMatrix(m,5,mxREAL);double* out=mxGetPr(plhs[0]);
        mwSize j=0;double left=0;
        for(mwSize i=0;i<m;++i) {
            require(s[i]>=-1e-9&&s[i]<=total+1e-9&&(i==0||s[i]>=s[i-1]),"Arc lengths must be sorted within the path.");
            while(j+1<u.size()&&s[i]>=left+std::abs(u[j].delta_s)-1e-12) {
                q=advance(q,u[j],std::abs(u[j].delta_s));left+=std::abs(u[j].delta_s);++j;
            }
            State z=u.empty()?q:advance(q,u[j],std::max(0.0,std::min(s[i]-left,std::abs(u[j].delta_s))));
            out[i]=z.x;out[i+m]=z.y;out[i+2*m]=z.theta;out[i+3*m]=z.kappa;out[i+4*m]=z.d;
        }
    } else if(std::strcmp(mode,"clearance")==0) {
        require(nrhs==5&&nlhs==1,"clearance requires poses, [front,rear,halfwidth], convex polygon cells, and positive cap.");
        const double* q=realArray(prhs[1]);mwSize n=mxGetM(prhs[1]);require(mxGetN(prhs[1])>=3,"Poses require at least three columns.");
        const double* g=realArray(prhs[2]);require(mxGetNumberOfElements(prhs[2])==3&&g[0]>0&&g[1]>0&&g[2]>0,"Invalid body dimensions.");
        require(mxIsCell(prhs[3]),"Obstacles must be a cell array.");
        const double* capValue=realArray(prhs[4]);require(mxGetNumberOfElements(prhs[4])==1&&capValue[0]>0,"Distance cap must be positive.");double cap=capValue[0];
        std::vector<std::vector<cphc::V>> obstacles;std::vector<cphc::V> centers;std::vector<double> radii;
        for(mwSize j=0;j<mxGetNumberOfElements(prhs[3]);++j){
            const mxArray* poly=mxGetCell(prhs[3],j);require(poly!=nullptr,"Empty obstacle cell.");const double* p=realArray(poly);mwSize m=mxGetM(poly);
            require(m>=3&&mxGetN(poly)==2,"Obstacle must be a convex N-by-2 polygon.");std::vector<cphc::V> vertices;cphc::V center{0,0};
            for(mwSize l=0;l<m;++l){vertices.push_back({p[l],p[l+m]});center=center+vertices.back()*(1.0/m);}
            double r=0;for(auto v:vertices)r=std::max(r,std::sqrt(cphc::dot(v-center,v-center)));
            obstacles.push_back(vertices);centers.push_back(center);radii.push_back(r);
        }
        double halfLength=(g[0]+g[1])/2,offset=(g[0]-g[1])/2,radius=std::hypot(halfLength,g[2]);
        plhs[0]=mxCreateDoubleMatrix(n,1,mxREAL);double* out=mxGetPr(plhs[0]);
        for(mwSize i=0;i<n;++i){
            double ct=std::cos(q[i+2*n]),st=std::sin(q[i+2*n]);cphc::V center{q[i]+offset*ct,q[i+n]+offset*st};
            std::vector<cphc::V> body;for(auto v:std::array<cphc::V,4>{{{g[0],g[2]},{g[0],-g[2]},{-g[1],-g[2]},{-g[1],g[2]}}})body.push_back({q[i]+ct*v.x-st*v.y,q[i+n]+st*v.x+ct*v.y});
            double gap=cap;
            for(mwSize j=0;j<obstacles.size();++j){
                cphc::V delta=center-centers[j];if(std::sqrt(cphc::dot(delta,delta))-radius-radii[j]>=gap)continue;
                gap=std::min(gap,cphc::distance(body,obstacles[j]));if(gap<=1e-12)break;
            }out[i]=gap;
        }
    } else throw std::runtime_error("Unknown mode.");
  } catch(const std::exception& e) {mexErrMsgIdAndTxt("CommonParking:HCSteer", "%s",e.what());}
}
