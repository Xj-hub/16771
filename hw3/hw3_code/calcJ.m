function j = calcJ(twists,theta)
ad = cell(1,6);
prod = eye(6);
prod_ad = cell(1,7);
prod_ad{1} = prod;
j = zeros(size(twists));

for i = 2:length(theta)
    ad{i-1} = adjoint(theta(i-1),twists(:,i-1));
    prod = prod * ad{i-1};
    prod_ad{i} = prod;
end
for i = 1:length(theta)
    j(:,i) = prod_ad{i} * twists(:,i);
end


end

