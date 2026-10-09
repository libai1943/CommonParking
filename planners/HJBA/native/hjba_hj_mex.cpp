// Independent fused implementation of the MATLAB ENO2/LF/SSP-RK3 stencil.
#include "mex.h"
#include <vector>
#include <cmath>
#include <algorithm>
struct Grid {
 size_t n[3],s[3]; double h[3]; std::vector<double> vx,vy;double omega;
 double get(const std::vector<double>& a,size_t at,int d,long q)const{
  long k=long((at/s[d])%n[d]);size_t base=at-size_t(k)*s[d];long N=long(n[d]);
  if(d==2){q=(q%N+N)%N;return a[base+size_t(q)*s[d]];}
  if(q<0)return (1-q)*a[base]+q*a[base+s[d]];
  if(q>=N){long j=q-N+1;return (1+j)*a[base+size_t(N-1)*s[d]]-j*a[base+size_t(N-2)*s[d]];}
  return a[base+size_t(q)*s[d]];
 }
 void derivative(const std::vector<double>& a,size_t at,int d,double& dm,double& dp)const{
  long k=long((at/s[d])%n[d]);double z=a[at],l=get(a,at,d,k-1),ll=get(a,at,d,k-2),r=get(a,at,d,k+1),rr=get(a,at,d,k+2),D=h[d];
  double dl=(z-2*l+ll)/(D*D),dc=(r-2*z+l)/(D*D),dr=(rr-2*r+z)/(D*D);
  dm=(z-l)/D+.5*D*(std::abs(dl)<std::abs(dc)?dl:dc);dp=(r-z)/D-.5*D*(std::abs(dr)<std::abs(dc)?dr:dc);
 }
 void stage(const std::vector<double>& a,const std::vector<double>& base,const double* target,std::vector<double>& out,double dt,double weight)const{
  for(size_t at=0;at<a.size();++at){double xm,xp,ym,yp,tm,tp;derivative(a,at,0,xm,xp);derivative(a,at,1,ym,yp);derivative(a,at,2,tm,tp);size_t k=at/s[2];
   double f=.5*(vx[k]*(xm+xp)+vy[k]*(ym+yp)-omega*std::abs(tm+tp)+std::abs(vx[k])*(xp-xm)+std::abs(vy[k])*(yp-ym)+omega*(tp-tm));
   out[at]=std::min(target[at],(1-weight)*base[at]+weight*(a[at]+dt*f));
  }
 }
};
#ifdef _WIN32
#define HJBA_EXPORT __declspec(dllexport)
#else
#define HJBA_EXPORT
#endif
extern "C" HJBA_EXPORT void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]){
 if(nrhs!=7||nlhs>2)mexErrMsgIdAndTxt("HJBA:arguments","Seven inputs and at most two outputs required.");
 for(int j=0;j<7;++j)if(!mxIsDouble(prhs[j])||mxIsComplex(prhs[j]))mexErrMsgIdAndTxt("HJBA:type","Inputs must be real doubles.");
 if(mxGetNumberOfDimensions(prhs[0])!=3||mxGetNumberOfElements(prhs[1])!=2)mexErrMsgIdAndTxt("HJBA:shape","Expected a 3D grid and two XY spacings.");
 Grid g;const mwSize* dims=mxGetDimensions(prhs[0]);for(int j=0;j<3;++j){g.n[j]=dims[j];if(g.n[j]<3)mexErrMsgIdAndTxt("HJBA:grid","Every grid dimension needs at least three points.");}
 if(mxGetNumberOfElements(prhs[2])!=g.n[2])mexErrMsgIdAndTxt("HJBA:heading","Heading dimension mismatch.");
 for(int j=3;j<7;++j)if(mxGetNumberOfElements(prhs[j])!=1)mexErrMsgIdAndTxt("HJBA:scalar","Scalar parameters required.");
 g.s[0]=1;g.s[1]=g.n[0];g.s[2]=g.n[0]*g.n[1];g.h[0]=mxGetPr(prhs[1])[0];g.h[1]=mxGetPr(prhs[1])[1];g.h[2]=2*std::acos(-1.)/g.n[2];double speed=mxGetScalar(prhs[3]);g.omega=mxGetScalar(prhs[4]);double horizon=mxGetScalar(prhs[5]),cfl=mxGetScalar(prhs[6]);
 if(!(g.h[0]>0&&g.h[1]>0&&g.omega>=0&&horizon>=0&&cfl>0&&cfl<=1))mexErrMsgIdAndTxt("HJBA:range","Invalid numerical parameters.");
 for(int j=0;j<7;++j)for(size_t k=0;k<mxGetNumberOfElements(prhs[j]);++k)if(!std::isfinite(mxGetPr(prhs[j])[k]))mexErrMsgIdAndTxt("HJBA:finite","All inputs must be finite.");
 double rate=0;for(size_t k=0;k<g.n[2];++k){double a=mxGetPr(prhs[2])[k];g.vx.push_back(speed*std::cos(a));g.vy.push_back(speed*std::sin(a));rate=std::max(rate,std::abs(g.vx.back())/g.h[0]+std::abs(g.vy.back())/g.h[1]+g.omega/g.h[2]);}
 size_t steps=std::max(size_t(1),size_t(std::ceil(horizon*rate/cfl)));double dt=horizon/steps;size_t N=mxGetNumberOfElements(prhs[0]);const double* target=mxGetPr(prhs[0]);std::vector<double> value(target,target+N),a(N),b(N),out(N);
 for(size_t j=0;j<steps;++j){g.stage(value,value,target,a,dt,1);g.stage(a,value,target,b,dt,.25);g.stage(b,value,target,out,dt,2./3);value.swap(out);}
 if(nlhs>0){plhs[0]=mxCreateNumericArray(3,dims,mxDOUBLE_CLASS,mxREAL);std::copy(value.begin(),value.end(),mxGetPr(plhs[0]));}if(nlhs>1)plhs[1]=mxCreateDoubleScalar(double(steps));
}
