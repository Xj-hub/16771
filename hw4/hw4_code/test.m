load pointsForLineFitting.mat
plot(points(:,1),points(:,2),'o');
hold on
modelLeastSquares = polyfit(points(:,1),points(:,2),1);
x = [min(points(:,1)) max(points(:,1))];
y = modelLeastSquares(1)*x + modelLeastSquares(2);
plot(x,y,'r-')
