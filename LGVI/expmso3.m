function R=expmso3(r);
nr=norm(r);
if nr==0;
    R=eye(3);
else
    Sr=[0 -r(3) r(2);r(3) 0 -r(1);-r(2) r(1) 0];
    R=eye(3)+sin(nr)/nr*Sr+2*sin(0.5*nr)^2/(nr)^2*Sr^2;
    tr = trace(R)
    anglee = acos( (trace(R) - 1)/2 )
end
