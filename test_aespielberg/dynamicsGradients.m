function [df, d2f, d3f] = dynamicsGradients(a1, a2, a3, a4, order)
% This is an auto-generated file.
%
% See <a href="matlab: help generateGradients">generateGradients</a>. 

% Check inputs:
typecheck(a1,'RigidBodyManipulator');
if (nargin<4) order=1; end
if (order<1) error(' order must be >= 1'); end
sizecheck(a1,[1  1]);
sizecheck(a2,[1  1]);
sizecheck(a3,[4  1]);
sizecheck(a4,[1  1]);

% Symbol table:
B_1=a1.B(1);
B_2=a1.B(2);
a3_1=a3(1);
a3_2=a3(2);
a3_3=a3(3);
a3_4=a3(4);
a4_1=a4(1);


% Compute Gradients:
df = sparse(4,6);
df(1,4) = 1;
df(2,5) = 1;
df(3,2) = (4905*cos(a3_1) - 981*cos(a3_1 + 2*a3_2))/(100*(cos(2*a3_2) - 8));
df(3,3) = - (981*cos(a3_1 + 2*a3_2) + 100*a3_3^2*cos(2*a3_2) - 10*a3_4*sin(a3_2) + 200*a3_3^2*cos(a3_2) + 200*a3_4^2*cos(a3_2) + 100*B_2*a4_1*sin(a3_2) + 400*a3_3*a3_4*cos(a3_2))/(50*(cos(2*a3_2) - 8)) - (sin(2*a3_2)*(20*a3_4 - 20*a3_3 - (4905*sin(a3_1))/2 + (981*sin(a3_1 + 2*a3_2))/2 + 200*B_1*a4_1 - 200*B_2*a4_1 + 50*a3_3^2*sin(2*a3_2) + 10*a3_4*cos(a3_2) + 200*a3_3^2*sin(a3_2) + 200*a3_4^2*sin(a3_2) - 100*B_2*a4_1*cos(a3_2) + 400*a3_3*a3_4*sin(a3_2)))/(25*(cos(2*a3_2) - 8)^2);
df(3,4) = -(40*a3_3*sin(a3_2) + 40*a3_4*sin(a3_2) + 20*a3_3*cos(a3_2)*sin(a3_2) - 2)/(5*(2*cos(a3_2)^2 - 9));
df(3,5) = -(cos(a3_2) + 40*a3_3*sin(a3_2) + 40*a3_4*sin(a3_2) + 2)/(5*(2*cos(a3_2)^2 - 9));
df(3,6) = (4*B_2 - 4*B_1 + 2*B_2*cos(a3_2))/(2*cos(a3_2)^2 - 9);
df(4,2) = (5886*cos(a3_1 + a3_2) - 9810*cos(a3_1) - 2943*cos(a3_1 - a3_2) + 1962*cos(a3_1 + 2*a3_2))/(200*(cos(2*a3_2) - 8));
df(4,3) = (5886*cos(a3_1 + a3_2) + 2943*cos(a3_1 - a3_2) + 3924*cos(a3_1 + 2*a3_2) + 800*a3_3^2*cos(2*a3_2) + 400*a3_4^2*cos(2*a3_2) + 40*a3_3*sin(a3_2) - 80*a3_4*sin(a3_2) + 1700*a3_3^2*cos(a3_2) + 800*a3_4^2*cos(a3_2) + 800*a3_3*a3_4*cos(2*a3_2) - 400*B_1*a4_1*sin(a3_2) + 800*B_2*a4_1*sin(a3_2) + 1600*a3_3*a3_4*cos(a3_2))/(200*(cos(2*a3_2) - 8)) + (sin(2*a3_2)*(170*a3_4 - 80*a3_3 + 5886*sin(a3_1 + a3_2) - 9810*sin(a3_1) - 2943*sin(a3_1 - a3_2) + 1962*sin(a3_1 + 2*a3_2) + 800*B_1*a4_1 - 1700*B_2*a4_1 + 400*a3_3^2*sin(2*a3_2) + 200*a3_4^2*sin(2*a3_2) - 40*a3_3*cos(a3_2) + 80*a3_4*cos(a3_2) + 1700*a3_3^2*sin(a3_2) + 800*a3_4^2*sin(a3_2) + 400*a3_3*a3_4*sin(2*a3_2) + 400*B_1*a4_1*cos(a3_2) - 800*B_2*a4_1*cos(a3_2) + 1600*a3_3*a3_4*sin(a3_2)))/(100*(cos(2*a3_2) - 8)^2);
df(4,4) = (85*a3_3*sin(a3_2) - cos(a3_2) + 40*a3_4*sin(a3_2) + 20*a3_3*sin(2*a3_2) + 10*a3_4*sin(2*a3_2) - 2)/(5*(cos(2*a3_2) - 8));
df(4,5) = (8*cos(a3_2) + 160*a3_3*sin(a3_2) + 160*a3_4*sin(a3_2) + 40*a3_3*sin(2*a3_2) + 40*a3_4*sin(2*a3_2) + 17)/(20*(cos(2*a3_2) - 8));
df(4,6) = (8*B_1 - 17*B_2 + 4*B_1*cos(a3_2) - 8*B_2*cos(a3_2))/(4*cos(a3_2)^2 - 18);

% d2f
if (order>=2)
  d2f = sparse(4,36);
  d2f(3,8) = -(4905*sin(a3_1) - 981*sin(a3_1 + 2*a3_2))/(100*(cos(2*a3_2) - 8));
  d2f(3,9) = -(7848*cos(2*a3_2)*sin(a3_1) - 981*sin(a3_1) + 2943*sin(2*a3_2)*cos(a3_1))/(50*(cos(2*a3_2) - 8)^2);
  d2f(3,14) = -(7848*cos(2*a3_2)*sin(a3_1) - 981*sin(a3_1) + 2943*sin(2*a3_2)*cos(a3_1))/(50*(cos(2*a3_2) - 8)^2);
  d2f(3,15) = (981*sin(a3_1 + 2*a3_2) + 100*a3_3^2*sin(2*a3_2) + 5*a3_4*cos(a3_2) + 100*a3_3^2*sin(a3_2) + 100*a3_4^2*sin(a3_2) - 50*B_2*a4_1*cos(a3_2) + 200*a3_3*a3_4*sin(a3_2))/(25*(cos(2*a3_2) - 8)) - (2*cos(2*a3_2)*(20*a3_4 - 20*a3_3 - (4905*sin(a3_1))/2 + (981*sin(a3_1 + 2*a3_2))/2 + 200*B_1*a4_1 - 200*B_2*a4_1 + 50*a3_3^2*sin(2*a3_2) + 10*a3_4*cos(a3_2) + 200*a3_3^2*sin(a3_2) + 200*a3_4^2*sin(a3_2) - 100*B_2*a4_1*cos(a3_2) + 400*a3_3*a3_4*sin(a3_2)))/(25*(cos(2*a3_2) - 8)^2) - (2*sin(2*a3_2)*(981*cos(a3_1 + 2*a3_2) + 100*a3_3^2*cos(2*a3_2) - 10*a3_4*sin(a3_2) + 200*a3_3^2*cos(a3_2) + 200*a3_4^2*cos(a3_2) + 100*B_2*a4_1*sin(a3_2) + 400*a3_3*a3_4*cos(a3_2)))/(25*(cos(2*a3_2) - 8)^2) - (4*sin(2*a3_2)^2*(20*a3_4 - 20*a3_3 - (4905*sin(a3_1))/2 + (981*sin(a3_1 + 2*a3_2))/2 + 200*B_1*a4_1 - 200*B_2*a4_1 + 50*a3_3^2*sin(2*a3_2) + 10*a3_4*cos(a3_2) + 200*a3_3^2*sin(a3_2) + 200*a3_4^2*sin(a3_2) - 100*B_2*a4_1*cos(a3_2) + 400*a3_3*a3_4*sin(a3_2)))/(25*(cos(2*a3_2) - 8)^3);
  d2f(3,16) = - (8*a3_3*cos(a3_2) + 8*a3_4*cos(a3_2) + 4*a3_3*(2*cos(a3_2)^2 - 1))/(2*cos(a3_2)^2 - 9) - (4*cos(a3_2)*sin(a3_2)*(40*a3_3*sin(a3_2) + 40*a3_4*sin(a3_2) + 20*a3_3*cos(a3_2)*sin(a3_2) - 2))/(5*(2*cos(a3_2)^2 - 9)^2);
  d2f(3,17) = - (40*a3_3*cos(a3_2) - sin(a3_2) + 40*a3_4*cos(a3_2))/(5*(2*cos(a3_2)^2 - 9)) - (4*cos(a3_2)*sin(a3_2)*(cos(a3_2) + 40*a3_3*sin(a3_2) + 40*a3_4*sin(a3_2) + 2))/(5*(2*cos(a3_2)^2 - 9)^2);
  d2f(3,18) = (2*sin(a3_2)*(9*B_2 - 8*B_1*cos(a3_2) + 8*B_2*cos(a3_2) + 2*B_2*cos(a3_2)^2))/(2*cos(a3_2)^2 - 9)^2;
  d2f(3,21) = - (8*a3_3*cos(a3_2) + 8*a3_4*cos(a3_2) + 4*a3_3*(2*cos(a3_2)^2 - 1))/(2*cos(a3_2)^2 - 9) - (2*cos(a3_2)*sin(a3_2)*(400*a3_3*sin(a3_2) + 400*a3_4*sin(a3_2) + 200*a3_3*cos(a3_2)*sin(a3_2) - 20))/(25*(2*cos(a3_2)^2 - 9)^2);
  d2f(3,22) = -(8*sin(a3_2) + 4*cos(a3_2)*sin(a3_2))/(2*cos(a3_2)^2 - 9);
  d2f(3,23) = (8*sin(a3_2))/(2*sin(a3_2)^2 + 7);
  d2f(3,27) = - (40*a3_3*cos(a3_2) - sin(a3_2) + 40*a3_4*cos(a3_2))/(5*(2*cos(a3_2)^2 - 9)) - (2*cos(a3_2)*sin(a3_2)*(10*cos(a3_2) + 400*a3_3*sin(a3_2) + 400*a3_4*sin(a3_2) + 20))/(25*(2*cos(a3_2)^2 - 9)^2);
  d2f(3,28) = (8*sin(a3_2))/(2*sin(a3_2)^2 + 7);
  d2f(3,29) = (8*sin(a3_2))/(2*sin(a3_2)^2 + 7);
  d2f(3,33) = (2*sin(a3_2)*(9*B_2 - 8*B_1*cos(a3_2) + 8*B_2*cos(a3_2) + 2*B_2*cos(a3_2)^2))/(2*cos(a3_2)^2 - 9)^2;
  d2f(4,8) = -(5886*sin(a3_1 + a3_2) - 9810*sin(a3_1) - 2943*sin(a3_1 - a3_2) + 1962*sin(a3_1 + 2*a3_2))/(200*(cos(2*a3_2) - 8));
  d2f(4,9) = (62784*cos(a3_2)^2*sin(a3_1) - 35316*sin(a3_1) + 17658*cos(a3_2)^3*sin(a3_1) + 26487*cos(a3_1)*sin(a3_2) + 44145*cos(a3_2)*sin(a3_1) + 5886*cos(a3_1)*cos(a3_2)^2*sin(a3_2) + 23544*cos(a3_1)*cos(a3_2)*sin(a3_2))/(200*(2*cos(a3_2)^2 - 9)^2);
  d2f(4,14) = (62784*cos(a3_2)^2*sin(a3_1) - 35316*sin(a3_1) + 17658*cos(a3_2)^3*sin(a3_1) + 26487*cos(a3_1)*sin(a3_2) + 44145*cos(a3_2)*sin(a3_1) + 5886*cos(a3_1)*cos(a3_2)^2*sin(a3_2) + 23544*cos(a3_1)*cos(a3_2)*sin(a3_2))/(200*(2*cos(a3_2)^2 - 9)^2);
  d2f(4,15) = -(1920*a3_3 - 4080*a3_4 - 141264*sin(a3_1) - 19200*B_1*a4_1 + 40800*B_2*a4_1 + 396800*a3_3^2*sin(2*a3_2) + 147900*a3_3^2*sin(3*a3_2) + 198400*a3_4^2*sin(2*a3_2) + 25600*a3_3^2*sin(4*a3_2) + 69600*a3_4^2*sin(3*a3_2) + 1700*a3_3^2*sin(5*a3_2) + 12800*a3_4^2*sin(4*a3_2) + 800*a3_4^2*sin(5*a3_2) + 753408*cos(2*a3_2)*sin(a3_1) + 1946304*sin(2*a3_2)*cos(a3_1) + 309015*cos(3*a3_2)*sin(a3_1) + 768123*sin(3*a3_2)*cos(a3_1) + 47088*cos(4*a3_2)*sin(a3_1) + 125568*sin(4*a3_2)*cos(a3_1) + 2943*cos(5*a3_2)*sin(a3_1) + 8829*sin(5*a3_2)*cos(a3_1) - 8080*a3_3*cos(a3_2) + 16160*a3_4*cos(a3_2) - 10240*a3_3*cos(2*a3_2) - 4200*a3_3*cos(3*a3_2) + 21760*a3_4*cos(2*a3_2) - 640*a3_3*cos(4*a3_2) + 8400*a3_4*cos(3*a3_2) - 40*a3_3*cos(5*a3_2) + 1360*a3_4*cos(4*a3_2) + 80*a3_4*cos(5*a3_2) + 452200*a3_3^2*sin(a3_2) + 212800*a3_4^2*sin(a3_2) + 2348514*cos(a3_1)*sin(a3_2) + 594486*cos(a3_2)*sin(a3_1) + 102400*B_1*a4_1*cos(2*a3_2) + 42000*B_1*a4_1*cos(3*a3_2) - 217600*B_2*a4_1*cos(2*a3_2) + 6400*B_1*a4_1*cos(4*a3_2) - 84000*B_2*a4_1*cos(3*a3_2) + 400*B_1*a4_1*cos(5*a3_2) - 13600*B_2*a4_1*cos(4*a3_2) - 800*B_2*a4_1*cos(5*a3_2) + 396800*a3_3*a3_4*sin(2*a3_2) + 139200*a3_3*a3_4*sin(3*a3_2) + 25600*a3_3*a3_4*sin(4*a3_2) + 1600*a3_3*a3_4*sin(5*a3_2) + 80800*B_1*a4_1*cos(a3_2) - 161600*B_2*a4_1*cos(a3_2) + 425600*a3_3*a3_4*sin(a3_2))/(800*(cos(2*a3_2) - 8)^3);
  d2f(4,16) = (sin(a3_2) + 85*a3_3*cos(a3_2) + 40*a3_4*cos(a3_2) + 40*a3_3*cos(2*a3_2) + 20*a3_4*cos(2*a3_2))/(5*(cos(2*a3_2) - 8)) + (2*sin(2*a3_2)*(85*a3_3*sin(a3_2) - cos(a3_2) + 40*a3_4*sin(a3_2) + 20*a3_3*sin(2*a3_2) + 10*a3_4*sin(2*a3_2) - 2))/(5*(cos(2*a3_2) - 8)^2);
  d2f(4,17) = (160*a3_3*cos(a3_2) - 8*sin(a3_2) + 160*a3_4*cos(a3_2) + 80*a3_3*cos(2*a3_2) + 80*a3_4*cos(2*a3_2))/(20*(cos(2*a3_2) - 8)) + (sin(2*a3_2)*(8*cos(a3_2) + 160*a3_3*sin(a3_2) + 160*a3_4*sin(a3_2) + 40*a3_3*sin(2*a3_2) + 40*a3_4*sin(2*a3_2) + 17))/(10*(cos(2*a3_2) - 8)^2);
  d2f(4,18) = (2*sin(a3_2)*(9*B_1 - 18*B_2 + 8*B_1*cos(a3_2) - 17*B_2*cos(a3_2) + 2*B_1*cos(a3_2)^2 - 4*B_2*cos(a3_2)^2))/(2*cos(a3_2)^2 - 9)^2;
  d2f(4,21) = (40*sin(a3_2) + 3400*a3_3*cos(a3_2) + 1600*a3_4*cos(a3_2) + 1600*a3_3*cos(2*a3_2) + 800*a3_4*cos(2*a3_2))/(200*(cos(2*a3_2) - 8)) + (sin(2*a3_2)*(3400*a3_3*sin(a3_2) - 40*cos(a3_2) + 1600*a3_4*sin(a3_2) + 800*a3_3*sin(2*a3_2) + 400*a3_4*sin(2*a3_2) - 80))/(100*(cos(2*a3_2) - 8)^2);
  d2f(4,22) = (17*sin(a3_2) + 8*cos(a3_2)*sin(a3_2))/(2*cos(a3_2)^2 - 9);
  d2f(4,23) = (8*sin(a3_2) + 4*cos(a3_2)*sin(a3_2))/(2*cos(a3_2)^2 - 9);
  d2f(4,27) = (1600*a3_3*cos(a3_2) - 80*sin(a3_2) + 1600*a3_4*cos(a3_2) + 800*a3_3*cos(2*a3_2) + 800*a3_4*cos(2*a3_2))/(200*(cos(2*a3_2) - 8)) + (sin(2*a3_2)*(80*cos(a3_2) + 1600*a3_3*sin(a3_2) + 1600*a3_4*sin(a3_2) + 400*a3_3*sin(2*a3_2) + 400*a3_4*sin(2*a3_2) + 170))/(100*(cos(2*a3_2) - 8)^2);
  d2f(4,28) = (8*sin(a3_2) + 4*cos(a3_2)*sin(a3_2))/(2*cos(a3_2)^2 - 9);
  d2f(4,29) = (8*sin(a3_2) + 4*cos(a3_2)*sin(a3_2))/(2*cos(a3_2)^2 - 9);
  d2f(4,33) = (2*sin(a3_2)*(9*B_1 - 18*B_2 + 8*B_1*cos(a3_2) - 17*B_2*cos(a3_2) + 2*B_1*cos(a3_2)^2 - 4*B_2*cos(a3_2)^2))/(2*cos(a3_2)^2 - 9)^2;
else
  d2f=[];
end  % if (order>=2)

% d3f
if (order>=3)
  d3f = sparse(4,216);
  d3f(3,44) = -(4905*cos(a3_1) - 981*cos(a3_1 + 2*a3_2))/(100*(cos(2*a3_2) - 8));
  d3f(3,45) = (981*cos(a3_1) - 7848*cos(2*a3_2)*cos(a3_1) + 2943*sin(2*a3_2)*sin(a3_1))/(50*(cos(2*a3_2) - 8)^2);
  d3f(3,50) = (981*cos(a3_1) - 7848*cos(2*a3_2)*cos(a3_1) + 2943*sin(2*a3_2)*sin(a3_1))/(50*(cos(2*a3_2) - 8)^2);
  d3f(3,51) = -(5886*cos(a3_1) - 23544*cos(2*a3_2)*cos(a3_1) + 60822*sin(2*a3_2)*sin(a3_1) - 2943*cos(2*a3_2)^2*cos(a3_1) + 7848*cos(2*a3_2)*sin(2*a3_2)*sin(a3_1))/(25*(cos(2*a3_2) - 8)^3);
  d3f(3,80) = (981*cos(a3_1) - 7848*cos(2*a3_2)*cos(a3_1) + 2943*sin(2*a3_2)*sin(a3_1))/(50*(cos(2*a3_2) - 8)^2);
  d3f(3,81) = - (5886*cos(2*a3_2)*cos(a3_1) - 15696*sin(2*a3_2)*sin(a3_1))/(50*(cos(2*a3_2) - 8)^2) - (2*sin(2*a3_2)*(7848*cos(2*a3_2)*sin(a3_1) - 981*sin(a3_1) + 2943*sin(2*a3_2)*cos(a3_1)))/(25*(cos(2*a3_2) - 8)^3);
  d3f(3,86) = - (5886*cos(2*a3_2)*cos(a3_1) - 15696*sin(2*a3_2)*sin(a3_1))/(50*(cos(2*a3_2) - 8)^2) - (2*sin(2*a3_2)*(7848*cos(2*a3_2)*sin(a3_1) - 981*sin(a3_1) + 2943*sin(2*a3_2)*cos(a3_1)))/(25*(cos(2*a3_2) - 8)^3);
  d3f(3,87) = -(777600*a3_3^2*cos(2*a3_2) - 1946304*cos(a3_1) + 549500*a3_3^2*cos(3*a3_2) + 201600*a3_3^2*cos(4*a3_2) + 549500*a3_4^2*cos(3*a3_2) + 34100*a3_3^2*cos(5*a3_2) + 3200*a3_3^2*cos(6*a3_2) + 34100*a3_4^2*cos(5*a3_2) + 100*a3_3^2*cos(7*a3_2) + 100*a3_4^2*cos(7*a3_2) + 7628256*cos(2*a3_2)*cos(a3_1) + 1977696*cos(4*a3_2)*cos(a3_1) + 31392*cos(6*a3_2)*cos(a3_1) - 2742876*sin(2*a3_2)*sin(a3_1) - 753408*sin(4*a3_2)*sin(a3_1) - 11772*sin(6*a3_2)*sin(a3_1) - 18505*a3_4*sin(a3_2) - 198400*a3_3^2 + 161100*a3_3^2*cos(a3_2) + 161100*a3_4^2*cos(a3_2) + 37280*a3_3*sin(2*a3_2) - 37280*a3_4*sin(2*a3_2) + 10240*a3_3*sin(4*a3_2) - 30195*a3_4*sin(3*a3_2) - 10240*a3_4*sin(4*a3_2) + 160*a3_3*sin(6*a3_2) - 1975*a3_4*sin(5*a3_2) - 160*a3_4*sin(6*a3_2) - 5*a3_4*sin(7*a3_2) - 372800*B_1*a4_1*sin(2*a3_2) + 372800*B_2*a4_1*sin(2*a3_2) - 102400*B_1*a4_1*sin(4*a3_2) + 301950*B_2*a4_1*sin(3*a3_2) + 102400*B_2*a4_1*sin(4*a3_2) - 1600*B_1*a4_1*sin(6*a3_2) + 19750*B_2*a4_1*sin(5*a3_2) + 1600*B_2*a4_1*sin(6*a3_2) + 50*B_2*a4_1*sin(7*a3_2) + 1099000*a3_3*a3_4*cos(3*a3_2) + 68200*a3_3*a3_4*cos(5*a3_2) + 200*a3_3*a3_4*cos(7*a3_2) + 185050*B_2*a4_1*sin(a3_2) + 322200*a3_3*a3_4*cos(a3_2))/(200*(cos(2*a3_2) - 8)^4);
  d3f(3,88) = (2660*a3_3*sin(a3_2) - 4*cos(4*a3_2) - 64*cos(2*a3_2) + 2660*a3_4*sin(a3_2) + 2480*a3_3*sin(2*a3_2) + 870*a3_3*sin(3*a3_2) + 160*a3_3*sin(4*a3_2) + 870*a3_4*sin(3*a3_2) + 10*a3_3*sin(5*a3_2) + 10*a3_4*sin(5*a3_2) + 12)/(5*(cos(2*a3_2) - 8)^3);
  d3f(3,89) = (96*cos(a3_2)^2 - 27*cos(a3_2) + 100*cos(a3_2)^3 + 32*cos(a3_2)^4 + 4*cos(a3_2)^5 + 1800*a3_3*sin(a3_2) + 1800*a3_4*sin(a3_2) + 3360*a3_3*cos(a3_2)^2*sin(a3_2) + 3360*a3_4*cos(a3_2)^2*sin(a3_2) + 160*a3_3*cos(a3_2)^4*sin(a3_2) + 160*a3_4*cos(a3_2)^4*sin(a3_2) - 72)/(5*(2*cos(a3_2)^2 - 9)^3);
  d3f(3,90) = -(144*B_1 - 144*B_2 - 54*B_2*cos(a3_2) - 192*B_1*cos(a3_2)^2 + 192*B_2*cos(a3_2)^2 - 64*B_1*cos(a3_2)^4 + 200*B_2*cos(a3_2)^3 + 64*B_2*cos(a3_2)^4 + 8*B_2*cos(a3_2)^5)/(2*cos(a3_2)^2 - 9)^3;
  d3f(3,93) = (2660*a3_3*sin(a3_2) - 4*cos(4*a3_2) - 64*cos(2*a3_2) + 2660*a3_4*sin(a3_2) + 2480*a3_3*sin(2*a3_2) + 870*a3_3*sin(3*a3_2) + 160*a3_3*sin(4*a3_2) + 870*a3_4*sin(3*a3_2) + 10*a3_3*sin(5*a3_2) + 10*a3_4*sin(5*a3_2) + 12)/(5*(cos(2*a3_2) - 8)^3);
  d3f(3,94) = (40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,95) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,99) = (96*cos(a3_2)^2 - 27*cos(a3_2) + 100*cos(a3_2)^3 + 32*cos(a3_2)^4 + 4*cos(a3_2)^5 + 1800*a3_3*sin(a3_2) + 1800*a3_4*sin(a3_2) + 3360*a3_3*cos(a3_2)^2*sin(a3_2) + 3360*a3_4*cos(a3_2)^2*sin(a3_2) + 160*a3_3*cos(a3_2)^4*sin(a3_2) + 160*a3_4*cos(a3_2)^4*sin(a3_2) - 72)/(5*(2*cos(a3_2)^2 - 9)^3);
  d3f(3,100) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,101) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,105) = -(144*B_1 - 144*B_2 - 54*B_2*cos(a3_2) - 192*B_1*cos(a3_2)^2 + 192*B_2*cos(a3_2)^2 - 64*B_1*cos(a3_2)^4 + 200*B_2*cos(a3_2)^3 + 64*B_2*cos(a3_2)^4 + 8*B_2*cos(a3_2)^5)/(2*cos(a3_2)^2 - 9)^3;
  d3f(3,123) = (2660*a3_3*sin(a3_2) - 4*cos(4*a3_2) - 64*cos(2*a3_2) + 2660*a3_4*sin(a3_2) + 2480*a3_3*sin(2*a3_2) + 870*a3_3*sin(3*a3_2) + 160*a3_3*sin(4*a3_2) + 870*a3_4*sin(3*a3_2) + 10*a3_3*sin(5*a3_2) + 10*a3_4*sin(5*a3_2) + 12)/(5*(cos(2*a3_2) - 8)^3);
  d3f(3,124) = - (8*cos(a3_2) + 8*cos(a3_2)^2 - 4)/(2*cos(a3_2)^2 - 9) - (4*cos(a3_2)*sin(a3_2)*(40*sin(a3_2) + 20*cos(a3_2)*sin(a3_2)))/(5*(2*cos(a3_2)^2 - 9)^2);
  d3f(3,125) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,129) = - (8*cos(a3_2) + 8*cos(a3_2)^2 - 4)/(2*cos(a3_2)^2 - 9) - (2*cos(a3_2)*sin(a3_2)*(400*sin(a3_2) + 200*cos(a3_2)*sin(a3_2)))/(25*(2*cos(a3_2)^2 - 9)^2);
  d3f(3,135) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,159) = (96*cos(a3_2)^2 - 27*cos(a3_2) + 100*cos(a3_2)^3 + 32*cos(a3_2)^4 + 4*cos(a3_2)^5 + 1800*a3_3*sin(a3_2) + 1800*a3_4*sin(a3_2) + 3360*a3_3*cos(a3_2)^2*sin(a3_2) + 3360*a3_4*cos(a3_2)^2*sin(a3_2) + 160*a3_3*cos(a3_2)^4*sin(a3_2) + 160*a3_4*cos(a3_2)^4*sin(a3_2) - 72)/(5*(2*cos(a3_2)^2 - 9)^3);
  d3f(3,160) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,161) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,165) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,171) = (8*cos(a3_2)*(2*cos(a3_2)^2 + 5))/(2*cos(a3_2)^2 - 9)^2;
  d3f(3,195) = -(144*B_1 - 144*B_2 - 54*B_2*cos(a3_2) - 192*B_1*cos(a3_2)^2 + 192*B_2*cos(a3_2)^2 - 64*B_1*cos(a3_2)^4 + 200*B_2*cos(a3_2)^3 + 64*B_2*cos(a3_2)^4 + 8*B_2*cos(a3_2)^5)/(2*cos(a3_2)^2 - 9)^3;
  d3f(4,44) = -(5886*cos(a3_1 + a3_2) - 9810*cos(a3_1) - 2943*cos(a3_1 - a3_2) + 1962*cos(a3_1 + 2*a3_2))/(200*(cos(2*a3_2) - 8));
  d3f(4,45) = -(35316*cos(a3_1) + 26487*sin(a3_1)*sin(a3_2) - 62784*cos(a3_1)*cos(a3_2)^2 - 17658*cos(a3_1)*cos(a3_2)^3 - 44145*cos(a3_1)*cos(a3_2) + 5886*cos(a3_2)^2*sin(a3_1)*sin(a3_2) + 23544*cos(a3_2)*sin(a3_1)*sin(a3_2))/(200*(2*cos(a3_2)^2 - 9)^2);
  d3f(4,50) = -(35316*cos(a3_1) + 26487*sin(a3_1)*sin(a3_2) - 62784*cos(a3_1)*cos(a3_2)^2 - 17658*cos(a3_1)*cos(a3_2)^3 - 44145*cos(a3_1)*cos(a3_2) + 5886*cos(a3_2)^2*sin(a3_1)*sin(a3_2) + 23544*cos(a3_2)*sin(a3_1)*sin(a3_2))/(200*(2*cos(a3_2)^2 - 9)^2);
  d3f(4,51) = (847584*cos(a3_1) + 1589220*sin(a3_1)*sin(a3_2) - 1130112*cos(a3_1)*cos(a3_2)^2 - 1177200*cos(a3_1)*cos(a3_2)^3 - 376704*cos(a3_1)*cos(a3_2)^4 - 47088*cos(a3_1)*cos(a3_2)^5 + 317844*cos(a3_1)*cos(a3_2) + 2966544*cos(a3_2)^2*sin(a3_1)*sin(a3_2) + 1004544*cos(a3_2)^3*sin(a3_1)*sin(a3_2) + 141264*cos(a3_2)^4*sin(a3_1)*sin(a3_2) + 3390336*cos(a3_2)*sin(a3_1)*sin(a3_2))/(800*(2*cos(a3_2)^2 - 9)^3);
  d3f(4,80) = -(35316*cos(a3_1) + 26487*sin(a3_1)*sin(a3_2) - 62784*cos(a3_1)*cos(a3_2)^2 - 17658*cos(a3_1)*cos(a3_2)^3 - 44145*cos(a3_1)*cos(a3_2) + 5886*cos(a3_2)^2*sin(a3_1)*sin(a3_2) + 23544*cos(a3_2)*sin(a3_1)*sin(a3_2))/(200*(2*cos(a3_2)^2 - 9)^2);
  d3f(4,81) = (981*(216*cos(a3_1) + 405*sin(a3_1)*sin(a3_2) - 288*cos(a3_1)*cos(a3_2)^2 - 300*cos(a3_1)*cos(a3_2)^3 - 96*cos(a3_1)*cos(a3_2)^4 - 12*cos(a3_1)*cos(a3_2)^5 + 81*cos(a3_1)*cos(a3_2) + 756*cos(a3_2)^2*sin(a3_1)*sin(a3_2) + 256*cos(a3_2)^3*sin(a3_1)*sin(a3_2) + 36*cos(a3_2)^4*sin(a3_1)*sin(a3_2) + 864*cos(a3_2)*sin(a3_1)*sin(a3_2)))/(200*(2*cos(a3_2)^2 - 9)^3);
  d3f(4,86) = (981*(216*cos(a3_1) + 405*sin(a3_1)*sin(a3_2) - 288*cos(a3_1)*cos(a3_2)^2 - 300*cos(a3_1)*cos(a3_2)^3 - 96*cos(a3_1)*cos(a3_2)^4 - 12*cos(a3_1)*cos(a3_2)^5 + 81*cos(a3_1)*cos(a3_2) + 756*cos(a3_2)^2*sin(a3_1)*sin(a3_2) + 256*cos(a3_2)^3*sin(a3_1)*sin(a3_2) + 36*cos(a3_2)^4*sin(a3_1)*sin(a3_2) + 864*cos(a3_2)*sin(a3_1)*sin(a3_2)))/(200*(2*cos(a3_2)^2 - 9)^3);
  d3f(4,87) = (12441600*a3_3^2*cos(2*a3_2) - 10892043*sin(a3_1)*sin(a3_2) - 3174400*a3_3*a3_4 - 15570432*cos(a3_1) + 9341500*a3_3^2*cos(3*a3_2) + 6220800*a3_4^2*cos(2*a3_2) + 3225600*a3_3^2*cos(4*a3_2) + 4396000*a3_4^2*cos(3*a3_2) + 579700*a3_3^2*cos(5*a3_2) + 1612800*a3_4^2*cos(4*a3_2) + 51200*a3_3^2*cos(6*a3_2) + 272800*a3_4^2*cos(5*a3_2) + 1700*a3_3^2*cos(7*a3_2) + 25600*a3_4^2*cos(6*a3_2) + 800*a3_4^2*cos(7*a3_2) + 61026048*cos(2*a3_2)*cos(a3_1) + 48515355*cos(3*a3_2)*cos(a3_1) + 15821568*cos(4*a3_2)*cos(a3_1) + 3010689*cos(5*a3_2)*cos(a3_1) + 251136*cos(6*a3_2)*cos(a3_1) + 8829*cos(7*a3_2)*cos(a3_1) - 21943008*sin(2*a3_2)*sin(a3_1) - 17772777*sin(3*a3_2)*sin(a3_1) - 6027264*sin(4*a3_2)*sin(a3_1) - 1162485*sin(5*a3_2)*sin(a3_1) - 94176*sin(6*a3_2)*sin(a3_1) - 2943*sin(7*a3_2)*sin(a3_1) + 148040*a3_3*sin(a3_2) - 296080*a3_4*sin(a3_2) - 3174400*a3_3^2 - 1587200*a3_4^2 + 2738700*a3_3^2*cos(a3_2) + 1288800*a3_4^2*cos(a3_2) + 14223519*cos(a3_1)*cos(a3_2) + 298240*a3_3*sin(2*a3_2) + 241560*a3_3*sin(3*a3_2) - 633760*a3_4*sin(2*a3_2) + 81920*a3_3*sin(4*a3_2) - 483120*a3_4*sin(3*a3_2) + 15800*a3_3*sin(5*a3_2) - 174080*a3_4*sin(4*a3_2) + 1280*a3_3*sin(6*a3_2) - 31600*a3_4*sin(5*a3_2) + 40*a3_3*sin(7*a3_2) - 2720*a3_4*sin(6*a3_2) - 80*a3_4*sin(7*a3_2) - 2982400*B_1*a4_1*sin(2*a3_2) - 2415600*B_1*a4_1*sin(3*a3_2) + 6337600*B_2*a4_1*sin(2*a3_2) - 819200*B_1*a4_1*sin(4*a3_2) + 4831200*B_2*a4_1*sin(3*a3_2) - 158000*B_1*a4_1*sin(5*a3_2) + 1740800*B_2*a4_1*sin(4*a3_2) - 12800*B_1*a4_1*sin(6*a3_2) + 316000*B_2*a4_1*sin(5*a3_2) - 400*B_1*a4_1*sin(7*a3_2) + 27200*B_2*a4_1*sin(6*a3_2) + 800*B_2*a4_1*sin(7*a3_2) + 12441600*a3_3*a3_4*cos(2*a3_2) + 8792000*a3_3*a3_4*cos(3*a3_2) + 3225600*a3_3*a3_4*cos(4*a3_2) + 545600*a3_3*a3_4*cos(5*a3_2) + 51200*a3_3*a3_4*cos(6*a3_2) + 1600*a3_3*a3_4*cos(7*a3_2) - 1480400*B_1*a4_1*sin(a3_2) + 2960800*B_2*a4_1*sin(a3_2) + 2577600*a3_3*a3_4*cos(a3_2))/(1600*(cos(2*a3_2) - 8)^4);
  d3f(4,88) = -(22610*a3_3*sin(a3_2) - 105*cos(3*a3_2) - 16*cos(4*a3_2) - cos(5*a3_2) - 202*cos(a3_2) - 256*cos(2*a3_2) + 10640*a3_4*sin(a3_2) + 19840*a3_3*sin(2*a3_2) + 7395*a3_3*sin(3*a3_2) + 9920*a3_4*sin(2*a3_2) + 1280*a3_3*sin(4*a3_2) + 3480*a3_4*sin(3*a3_2) + 85*a3_3*sin(5*a3_2) + 640*a3_4*sin(4*a3_2) + 40*a3_4*sin(5*a3_2) + 48)/(20*(cos(2*a3_2) - 8)^3);
  d3f(4,89) = -(544*cos(2*a3_2) + 210*cos(3*a3_2) + 34*cos(4*a3_2) + 2*cos(5*a3_2) + 404*cos(a3_2) + 10640*a3_3*sin(a3_2) + 10640*a3_4*sin(a3_2) + 9920*a3_3*sin(2*a3_2) + 3480*a3_3*sin(3*a3_2) + 9920*a3_4*sin(2*a3_2) + 640*a3_3*sin(4*a3_2) + 3480*a3_4*sin(3*a3_2) + 40*a3_3*sin(5*a3_2) + 640*a3_4*sin(4*a3_2) + 40*a3_4*sin(5*a3_2) - 102)/(20*(cos(2*a3_2) - 8)^3);
  d3f(4,90) = (144*B_1 - 306*B_2 + 54*B_1*cos(a3_2) - 108*B_2*cos(a3_2) - 192*B_1*cos(a3_2)^2 - 200*B_1*cos(a3_2)^3 + 408*B_2*cos(a3_2)^2 - 64*B_1*cos(a3_2)^4 + 400*B_2*cos(a3_2)^3 - 8*B_1*cos(a3_2)^5 + 136*B_2*cos(a3_2)^4 + 16*B_2*cos(a3_2)^5)/(2*cos(a3_2)^2 - 9)^3;
  d3f(4,93) = -(22610*a3_3*sin(a3_2) - 105*cos(3*a3_2) - 16*cos(4*a3_2) - cos(5*a3_2) - 202*cos(a3_2) - 256*cos(2*a3_2) + 10640*a3_4*sin(a3_2) + 19840*a3_3*sin(2*a3_2) + 7395*a3_3*sin(3*a3_2) + 9920*a3_4*sin(2*a3_2) + 1280*a3_3*sin(4*a3_2) + 3480*a3_4*sin(3*a3_2) + 85*a3_3*sin(5*a3_2) + 640*a3_4*sin(4*a3_2) + 40*a3_4*sin(5*a3_2) + 48)/(20*(cos(2*a3_2) - 8)^3);
  d3f(4,94) = -(85*cos(a3_2) + 128*cos(a3_2)^2 + 34*cos(a3_2)^3 - 72)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,95) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,99) = -(544*cos(2*a3_2) + 210*cos(3*a3_2) + 34*cos(4*a3_2) + 2*cos(5*a3_2) + 404*cos(a3_2) + 10640*a3_3*sin(a3_2) + 10640*a3_4*sin(a3_2) + 9920*a3_3*sin(2*a3_2) + 3480*a3_3*sin(3*a3_2) + 9920*a3_4*sin(2*a3_2) + 640*a3_3*sin(4*a3_2) + 3480*a3_4*sin(3*a3_2) + 40*a3_3*sin(5*a3_2) + 640*a3_4*sin(4*a3_2) + 40*a3_4*sin(5*a3_2) - 102)/(20*(cos(2*a3_2) - 8)^3);
  d3f(4,100) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,101) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,105) = (144*B_1 - 306*B_2 + 54*B_1*cos(a3_2) - 108*B_2*cos(a3_2) - 192*B_1*cos(a3_2)^2 - 200*B_1*cos(a3_2)^3 + 408*B_2*cos(a3_2)^2 - 64*B_1*cos(a3_2)^4 + 400*B_2*cos(a3_2)^3 - 8*B_1*cos(a3_2)^5 + 136*B_2*cos(a3_2)^4 + 16*B_2*cos(a3_2)^5)/(2*cos(a3_2)^2 - 9)^3;
  d3f(4,123) = -(22610*a3_3*sin(a3_2) - 105*cos(3*a3_2) - 16*cos(4*a3_2) - cos(5*a3_2) - 202*cos(a3_2) - 256*cos(2*a3_2) + 10640*a3_4*sin(a3_2) + 19840*a3_3*sin(2*a3_2) + 7395*a3_3*sin(3*a3_2) + 9920*a3_4*sin(2*a3_2) + 1280*a3_3*sin(4*a3_2) + 3480*a3_4*sin(3*a3_2) + 85*a3_3*sin(5*a3_2) + 640*a3_4*sin(4*a3_2) + 40*a3_4*sin(5*a3_2) + 48)/(20*(cos(2*a3_2) - 8)^3);
  d3f(4,124) = -(85*cos(a3_2) + 128*cos(a3_2)^2 + 34*cos(a3_2)^3 - 72)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,125) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,129) = -(85*cos(a3_2) + 128*cos(a3_2)^2 + 34*cos(a3_2)^3 - 72)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,135) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,159) = -(272*cos(2*a3_2) + 105*cos(3*a3_2) + 17*cos(4*a3_2) + cos(5*a3_2) + 202*cos(a3_2) + 5320*a3_3*sin(a3_2) + 5320*a3_4*sin(a3_2) + 4960*a3_3*sin(2*a3_2) + 1740*a3_3*sin(3*a3_2) + 4960*a3_4*sin(2*a3_2) + 320*a3_3*sin(4*a3_2) + 1740*a3_4*sin(3*a3_2) + 20*a3_3*sin(5*a3_2) + 320*a3_4*sin(4*a3_2) + 20*a3_4*sin(5*a3_2) - 51)/(10*(cos(2*a3_2) - 8)^3);
  d3f(4,160) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,161) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,165) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,171) = -(40*cos(a3_2) + 64*cos(a3_2)^2 + 16*cos(a3_2)^3 - 36)/(2*cos(a3_2)^2 - 9)^2;
  d3f(4,195) = (144*B_1 - 306*B_2 + 54*B_1*cos(a3_2) - 108*B_2*cos(a3_2) - 192*B_1*cos(a3_2)^2 - 200*B_1*cos(a3_2)^3 + 408*B_2*cos(a3_2)^2 - 64*B_1*cos(a3_2)^4 + 400*B_2*cos(a3_2)^3 - 8*B_1*cos(a3_2)^5 + 136*B_2*cos(a3_2)^4 + 16*B_2*cos(a3_2)^5)/(2*cos(a3_2)^2 - 9)^3;
else
  d3f=[];
end  % if (order>=3)


 % NOTEST
