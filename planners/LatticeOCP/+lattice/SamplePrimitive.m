function z=SamplePrimitive(primitive,s,vehicle,gamma)
s=min(max(s(:),0),primitive.length);N=numel(primitive.controls);step=primitive.length/N;
index=min(N,floor(s/step)+1);distance=s-(index-1)*step;
z=lattice.Step(primitive.states(index,:),primitive.controls(index),distance,primitive.gear,vehicle,gamma);
z(s==primitive.length,:)=repmat(primitive.states(end,:),nnz(s==primitive.length),1);
end
