function [f]=trans_control_f(R,bt,nu,vd,dvd)

global L e3 g m P
% parameter

f = e3'*R'*(m*g*e3+P*bt+L*(R*nu-vd)-m*dvd);
%f=m*g*e3+P*bt+L*(R*nu-vd)-m*dvd;
