function Yhat=FTSobs(Ymeas,Ymeas0,Yhat0);
% Function file to compute model-free estimates from FTS  
% discrete-time observer for FTS model-free control

global L beta p
n=length(Ymeas);

% Observer gains
L=2.1*eye(n); 
gam=1.4;p=9/7;
ep=1-1/p;

% Initial state estimate error
Eo0=Yhat0-Ymeas0;

% Compute subsequent output estimate through FTS observer design
eok= (Eo0'*L*Eo0)^ep;
Bcal=(eok-gam)/(eok+gam);
Eo=Bcal*Eo0;
Yhat=Ymeas+Eo;
