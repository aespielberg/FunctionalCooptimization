function [f,df] = endEffectorConstraint(in1)
%ENDEFFECTORCONSTRAINT
%    [F,DF] = ENDEFFECTORCONSTRAINT(IN1)

%    This function was generated by the Symbolic Math Toolbox version 6.2.
%    10-Oct-2015 14:44:55

length1 = in1(5,:);
length2 = in1(6,:);
q1 = in1(1,:);
q2 = in1(2,:);
t2 = sin(q1);
t3 = cos(q1);
t4 = cos(q2);
t5 = sin(q2);
t6 = t3.*t4.*1.0;
t16 = t2.*t5;
t7 = t6-t16+3.749399456654644e-33;
t8 = t2.*t5.*6.123233995736766e-17;
t18 = t3.*t4.*6.123233995736766e-17;
t9 = t8-t18+6.123233995736766e-17;
t10 = conj(q1);
t11 = cos(t10);
t12 = conj(q2);
t13 = cos(t12);
t14 = sin(t10);
t15 = sin(t12);
t17 = t7.^2;
t19 = t9.^2;
t20 = t17+t19;
t21 = t3.*t5;
t22 = t2.*t4.*1.0;
t23 = t21+t22;
t24 = 1.0./t20;
t25 = sqrt(t20);
t26 = t3.*t4;
f = [t2.*6.123233995736766e-18+length1.*t2+length2.*t23.*(1.0./2.0);t3.*(-3.749399456654644e-34)+length2.*t9.*(1.0./2.0)-length1.*(t3.*6.123233995736766e-17-6.123233995736766e-17)-1.0./4.0;t3.*6.123233995736766e-18+length2.*t7.*(1.0./2.0)+length1.*(t3+3.749399456654644e-33)+1.0e1;-angle(t2.*t5.*(-1.0+6.123233995736766e-17i)+t3.*t4.*(1.0-6.123233995736766e-17i)+3.749399456654644e-33+6.123233995736766e-17i);-angle(t25+t2.*t4.*1i+t3.*t5.*1.0i);-angle(t26+t2.*t4.*6.123233995736766e-17i-t2.*t5.*1.0+t3.*t5.*6.123233995736766e-17i)];
if nargout > 1
    t27 = t3.*t5.*6.123233995736766e-17;
    t28 = t2.*t4.*6.123233995736766e-17;
    t29 = t27+t28;
    t30 = t3.*t5.*1.0;
    t31 = t2.*t4;
    t32 = t30+t31;
    t33 = 1.0./sqrt(t20);
    t34 = t3.*t5.*6.123233995736766e-17;
    t35 = t2.*t4.*6.123233995736766e-17;
    t36 = t34+t35;
    t37 = t32.^2;
    t38 = t17+t19+t37;
    t39 = 1.0./t38;
    t41 = t2.*t5.*1.0;
    t40 = t26-t41;
    t42 = t40.^2;
    t43 = t36.^2;
    t44 = t42+t43;
    t45 = 1.0./t44;
    df = reshape([t11.*6.123233995736766e-18+length2.*(t11.*t13.*1.0-t14.*t15).*(1.0./2.0)+length1.*t11,t14.*3.749399456654644e-34+length1.*t14.*6.123233995736766e-17+length2.*(t11.*t15.*6.123233995736766e-17+t13.*t14.*6.123233995736766e-17).*(1.0./2.0),t14.*(-6.123233995736766e-18)-length2.*(t11.*t15+t13.*t14.*1.0).*(1.0./2.0)-length1.*t14,-t24.*(t9.*t23+t7.*t29),-t39.*(t25.*t40+t32.*t33.*(t7.*t23-t9.*t29)),-t45.*(t32.*t36-t40.*(t2.*t5.*6.123233995736766e-17-t3.*t4.*6.123233995736766e-17)),length2.*(t11.*t13-t14.*t15.*1.0).*(1.0./2.0),length2.*(t11.*t15.*6.123233995736766e-17+t13.*t14.*6.123233995736766e-17).*(1.0./2.0),length2.*(t11.*t15.*1.0+t13.*t14).*(-1.0./2.0),-t24.*(t9.*t32+t7.*t36),-t39.*(t25.*(t6-t16)+t32.*t33.*(t7.*t32-t9.*t36)),-t45.*(t23.*t36-t40.*(t8-t18))],[6, 2]);
end
