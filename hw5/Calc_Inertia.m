function I = Calc_Inertia(m,l,r)
%Input m is mass, l is the length of the cylinder,
%r is the radius of cylinder
%Outpu the moment of inertia of cylinder wrt x,y,z axis

Ix = (m * r^2)/2;
Iy = m*(3*r^2 + l^2)/12;
Iz = Iy;
I = [Ix,Iy,Iz];
end

