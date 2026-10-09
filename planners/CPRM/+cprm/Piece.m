function p=Piece(type,direction)
p=struct('type',type,'direction',direction,'coefficients',zeros(4,2),'start',zeros(1,3),'primitive',zeros(1,2), ...
 'length',NaN,'maximum_curvature',NaN,'minimum_speed',NaN,'table_t',[],'table_s',[]);
end
