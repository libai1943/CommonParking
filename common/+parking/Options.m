function opt=Options(defaults,user)
opt=defaults;if nargin<2||isempty(user),return;end
for name=fieldnames(user)',opt.(name{1})=user.(name{1});end
end
