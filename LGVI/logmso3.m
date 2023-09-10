% For the matrix logarithm on SO(3) we use the 
% expression (5.9) in "Geometric Control of Mechanical Systems" 
% by F. Bullo and A. D. Lewis.

% Modifications made by AKS on Aug 5, 2011 to account for 
% rotations by an odd multiple of pi radians

function r=logmso3(R)
trR=trace(R);

phi=acos(0.5*(trR-1));
A=R-R';
nA=norm(A,'fro');
S=0.5*(R-eye(3));
sd=diag(S);
nS=norm(S,'fro');
[msd,i]=max(sd);

if nA < 1e-5 && nS < 1e-5, % This is when R is close to identity (phi is even multiple of pi rads)
    r=zeros(3,1);
elseif nA < 1e-5 && nS > 0.1, % This is when phi is an odd multiple of pi rads (logmso3 "strictly" singular)
    ri=sqrt(1+msd);
    if i==1, 
        r=pi*[ri;S(1,2)/ri;S(1,3)/ri];
    elseif i==2,
        r=pi*[S(2,1)/ri;ri;S(2,3)/ri];
    else r=pi*[S(3,1)/ri;S(3,2)/ri;ri];
    end;
else
    logR=cs(phi)*(R-R');
    r=unskew(logR);
end;

function y=cs(x)
if abs(x)<0.2
    y=1/2+x^2/12+(7*x^4)/720+(31*x^6)/30240+(127*x^8)/1209600+(73*x^10)/6842880+(1414477*x^12)/1307674368000;
else
    y=0.5*x/sin(x);
end

function w = unskew(Sw)
w=[Sw(3,2);Sw(1,3);Sw(2,1)];
