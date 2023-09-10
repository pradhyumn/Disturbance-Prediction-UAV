function rpm=thrust2rpm(thrust)


x=thrust;
%Coefficients:
  p1 = 0.022616;
  p2 = -0.84926;
  p3 = 13.534;
  p4 = -119.45;
  p5 = 639.47;
  p6 = -2140;
  p7 = 4463.6;
  p8 = -5704.4;
  p9 = 5260.6;
  p10 = 592.98;



rpm =p1*x^9 + p2*x^8 +...
      p3*x^7 + p4*x^6 +...
      p5*x^5 + p6*x^4 +...
      p7*x^3 + p8*x^2 +...
      p9*x + p10 ; 
end


