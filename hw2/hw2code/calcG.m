function gab = calcG(twists,angles,init)
%CALCG 此处显示有关此函数的摘要
%   此处显示详细说明
gab = init;
for ii = size(twists,2):-1:1
    gab = twistExp(twists(:,ii), angles(ii))*gab;
end
end

