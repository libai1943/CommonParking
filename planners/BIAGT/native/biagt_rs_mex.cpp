// Batch Reeds--Shepp geometry gateway. Search and mode logic are MATLAB.
#include "mex.h"
#include <cmath>
#include <cstring>
#include <stdexcept>
#include "steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp"
static void need(bool b,const char* s){if(!b)throw std::runtime_error(s);}
static const double* values(const mxArray* a){need(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a),"Full real doubles required.");const double*p=mxGetPr(a);for(mwSize i=0;i<mxGetNumberOfElements(a);++i)need(std::isfinite(p[i]),"Finite values required.");return p;}
static steering::State row(const mxArray*a,mwSize i){const double*p=mxGetPr(a);mwSize n=mxGetM(a);need(n>0&&mxGetN(a)==3,"Expected N by 3 poses.");i=n==1?0:i;need(i<n,"Batch mismatch.");return steering::State{p[i],p[i+n],p[i+2*n],0,0};}
#ifdef _WIN32
#define CP_EXPORT __declspec(dllexport)
#else
#define CP_EXPORT
#endif
extern "C" CP_EXPORT void mexFunction(int nlhs,mxArray* out[],int nrhs,const mxArray* in[]){try{
 need(nrhs==3&&nlhs>=1&&nlhs<=2,"Usage: [lengths,arcs]=biagt_rs_mex(q1,q2,kappa).");const double*k=values(in[2]);need(mxGetNumberOfElements(in[2])==1&&k[0]>0,"Positive curvature required.");
 values(in[0]);values(in[1]);mwSize n1=mxGetM(in[0]),n2=mxGetM(in[1]),n=std::max(n1,n2);need(n1==n2||n1==1||n2==1,"Batch mismatch.");steering::Reeds_Shepp_State_Space rs(k[0]);
 out[0]=mxCreateDoubleMatrix(n,1,mxREAL);if(nlhs>1)out[1]=mxCreateCellMatrix(n,1);
 for(mwSize i=0;i<n;++i){auto a=row(in[0],i),b=row(in[1],i);mxGetPr(out[0])[i]=rs.get_distance(a,b);
 if(nlhs>1){auto raw=rs.get_controls(a,b);std::vector<steering::Control>u;for(auto c:raw)if(std::abs(c.delta_s)>1e-12)u.push_back(c);mxArray*m=mxCreateDoubleMatrix(u.size(),2,mxREAL);double*p=mxGetPr(m);for(mwSize j=0;j<u.size();++j){p[j]=u[j].delta_s;p[j+u.size()]=u[j].kappa;}mxSetCell(out[1],i,m);}}
 }catch(const std::exception&e){mexErrMsgIdAndTxt("CommonParking:BIAGTRS","%s",e.what());}}
