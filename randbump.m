function x=randbump(c,m,n)
% This function generates a bump function with normal distribution
% "c" is the maximum value of the noise
% "m,n" are the dimensions of the output noise matrix
% MI 10/08/2014
x=zeros(m,n);
if c~=0
    for i=1:m
        for j=1:n
            k=0;
            while k==0
                x1=c*(2*rand(1)-1);
                x2=rand(1);
                if x2<exp(1-c^2/(c^2-x1^2))
                    x(i,j)=x1;
                    k=1;
                end
            end
        end
    end
end