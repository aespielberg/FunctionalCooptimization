function dynamicsEquation = dynamics_functionInertial(h1,in2,u01,in4)
%DYNAMICS_FUNCTIONINERTIAL
%    DYNAMICSEQUATION = DYNAMICS_FUNCTIONINERTIAL(H1,IN2,U01,IN4)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    29-Jun-2016 14:39:30

l1 = in4(1,:);
l2 = in4(2,:);
m1 = in4(3,:);
m2 = in4(4,:);
x01 = in2(1,:);
x02 = in2(2,:);
x03 = in2(3,:);
x04 = in2(4,:);
dynamicsEquation = [x03;x04;(u01.*1.2e1-x03.*(6.0./5.0)-((l2.*m2.*(1.0./2.0)+m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(l1.*x04.^2.*cos(x01)-l1.*x04.*cos(x01).*(x03+x04)+9.81e2./1.0e2).*1.2e1+(l1.*x04.^2.*sin(x01)-l1.*x04.*sin(x01).*(x03+x04)).*((l2.*m2.*(1.0./2.0)+m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*1.2e1-l1.*m1.*sin(x01).*5.886e1+l1.*x04.*cos(x01).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l2.*m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).*(1.0./2.0))-l1.*x04.*sin(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2)).*1.2e1-l1.*x04.*sin(x01).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))+l2.*m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).*(1.0./2.0))-l1.*x04.*cos(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2)).*1.2e1)./(l1.^2.*m1.*4.0+l1.^2.*m2.*cos(x01).^4.*cos(x02).^2.*3.0+l1.^2.*m2.*cos(x02).^2.*sin(x01).^4.*3.0+l1.^2.*m2.*cos(x01).^4.*sin(x02).^2.*1.2e1+l1.^2.*m2.*sin(x01).^4.*sin(x02).^2.*1.2e1+l1.^2.*m2.*cos(x01).^2.*cos(x02).^2.*sin(x01).^2.*6.0+l1.^2.*m2.*cos(x01).^2.*sin(x01).^2.*sin(x02).^2.*2.4e1)+((l2.*2.0+l1.*cos(x01).^2.*cos(x02).*3.0+l1.*cos(x02).*sin(x01).^2.*3.0).*(x04.*(1.0./1.0e1)+((l2.*m2.*(1.0./2.0)+m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(l1.*x04.^2.*cos(x01)-l1.*x04.*cos(x01).*(x03+x04)+9.81e2./1.0e2)-(l1.*x04.^2.*sin(x01)-l1.*x04.*sin(x01).*(x03+x04)).*((l2.*m2.*(1.0./2.0)+m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))-l1.*sin(x01).*((x03+x04).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))+l2.*m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).*(1.0./2.0))-l1.*x04.*cos(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))+(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2).*(l1.*x04.^2.*cos(x01)-l1.*x04.*cos(x01).*(x03+x04)+9.81e2./1.0e2))+l1.*cos(x01).*((x03+x04).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l2.*m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).*(1.0./2.0))-l1.*x04.*sin(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))+(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2).*(l1.*x04.^2.*sin(x01)-l1.*x04.*sin(x01).*(x03+x04)))-l1.*x04.*cos(x01).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l2.*m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).*(1.0./2.0))-l1.*x04.*sin(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))+l1.*x04.*sin(x01).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))+l2.*m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).*(1.0./2.0))-l1.*x04.*cos(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))).*6.0)./(l1.^2.*l2.*m1.*4.0+l1.^2.*l2.*m2.*cos(x01).^4.*cos(x02).^2.*3.0+l1.^2.*l2.*m2.*cos(x02).^2.*sin(x01).^4.*3.0+l1.^2.*l2.*m2.*cos(x01).^4.*sin(x02).^2.*1.2e1+l1.^2.*l2.*m2.*sin(x01).^4.*sin(x02).^2.*1.2e1+l1.^2.*l2.*m2.*cos(x01).^2.*cos(x02).^2.*sin(x01).^2.*6.0+l1.^2.*l2.*m2.*cos(x01).^2.*sin(x01).^2.*sin(x02).^2.*2.4e1);((x04.*(1.0./1.0e1)+((l2.*m2.*(1.0./2.0)+m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(l1.*x04.^2.*cos(x01)-l1.*x04.*cos(x01).*(x03+x04)+9.81e2./1.0e2)-(l1.*x04.^2.*sin(x01)-l1.*x04.*sin(x01).*(x03+x04)).*((l2.*m2.*(1.0./2.0)+m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))-l1.*sin(x01).*((x03+x04).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))+l2.*m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).*(1.0./2.0))-l1.*x04.*cos(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))+(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2).*(l1.*x04.^2.*cos(x01)-l1.*x04.*cos(x01).*(x03+x04)+9.81e2./1.0e2))+l1.*cos(x01).*((x03+x04).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l2.*m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).*(1.0./2.0))-l1.*x04.*sin(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))+(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2).*(l1.*x04.^2.*sin(x01)-l1.*x04.*sin(x01).*(x03+x04)))-l1.*x04.*cos(x01).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l2.*m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).*(1.0./2.0))-l1.*x04.*sin(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))+l1.*x04.*sin(x01).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))+l2.*m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).*(1.0./2.0))-l1.*x04.*cos(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))).*(l1.^2.*m1+l2.^2.*m2+l1.^2.*m2.*cos(x01).^4.*cos(x02).^2.*3.0+l1.^2.*m2.*cos(x02).^2.*sin(x01).^4.*3.0+l1.^2.*m2.*cos(x01).^4.*sin(x02).^2.*3.0+l1.^2.*m2.*sin(x01).^4.*sin(x02).^2.*3.0+l1.^2.*m2.*cos(x01).^2.*cos(x02).^2.*sin(x01).^2.*6.0+l1.^2.*m2.*cos(x01).^2.*sin(x01).^2.*sin(x02).^2.*6.0+l1.*l2.*m2.*cos(x01).^2.*cos(x02).*3.0+l1.*l2.*m2.*cos(x02).*sin(x01).^2.*3.0).*-1.2e1)./(l1.^2.*l2.^2.*m1.*m2.*4.0+l1.^2.*l2.^2.*m2.^2.*cos(x01).^4.*cos(x02).^2.*3.0+l1.^2.*l2.^2.*m2.^2.*cos(x02).^2.*sin(x01).^4.*3.0+l1.^2.*l2.^2.*m2.^2.*cos(x01).^4.*sin(x02).^2.*1.2e1+l1.^2.*l2.^2.*m2.^2.*sin(x01).^4.*sin(x02).^2.*1.2e1+l1.^2.*l2.^2.*m2.^2.*cos(x01).^2.*cos(x02).^2.*sin(x01).^2.*6.0+l1.^2.*l2.^2.*m2.^2.*cos(x01).^2.*sin(x01).^2.*sin(x02).^2.*2.4e1)+((l2.*2.0+l1.*cos(x01).^2.*cos(x02).*3.0+l1.*cos(x02).*sin(x01).^2.*3.0).*(-u01+x03.*(1.0./1.0e1)+((l2.*m2.*(1.0./2.0)+m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(l1.*x04.^2.*cos(x01)-l1.*x04.*cos(x01).*(x03+x04)+9.81e2./1.0e2)-(l1.*x04.^2.*sin(x01)-l1.*x04.*sin(x01).*(x03+x04)).*((l2.*m2.*(1.0./2.0)+m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01)))+l1.*m1.*sin(x01).*(9.81e2./2.0e2)-l1.*x04.*cos(x01).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l2.*m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).*(1.0./2.0))-l1.*x04.*sin(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))+l1.*x04.*sin(x01).*((x03+x04).*(m2.*(l1.*cos(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+l1.*sin(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))+m2.*(l1.*cos(x01).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))-l1.*sin(x01).*(cos(x01).*cos(x02)-sin(x01).*sin(x02))).*(cos(x01).*sin(x02)+cos(x02).*sin(x01))+l2.*m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).*(1.0./2.0))-l1.*x04.*cos(x01).*(m2.*(cos(x01).*sin(x02)+cos(x02).*sin(x01)).^2+m2.*(cos(x01).*cos(x02)-sin(x01).*sin(x02)).^2))).*6.0)./(l1.^2.*l2.*m1.*4.0+l1.^2.*l2.*m2.*cos(x01).^4.*cos(x02).^2.*3.0+l1.^2.*l2.*m2.*cos(x02).^2.*sin(x01).^4.*3.0+l1.^2.*l2.*m2.*cos(x01).^4.*sin(x02).^2.*1.2e1+l1.^2.*l2.*m2.*sin(x01).^4.*sin(x02).^2.*1.2e1+l1.^2.*l2.*m2.*cos(x01).^2.*cos(x02).^2.*sin(x01).^2.*6.0+l1.^2.*l2.*m2.*cos(x01).^2.*sin(x01).^2.*sin(x02).^2.*2.4e1)];
