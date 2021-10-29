function dx =Inv_Pdlm(x, u)

    global M m l g
    M = 5;m = 1;
    l = 0.5;g = 9.81;
    
    dx(1) = x(2);
    dx(2) = 1/(m+M-m*cos(x(3)))*(u-m*l*(x(4))^2*sin(x(3))-m*g*sin(x(3)));
    dx(3) = x(4);
    dx(4) = (g*sin(x(3))-(1/(m+M-m*cos(x(3)))*(u-m*l*(x(4))^2*sin(x(3))-m*g*sin(x(3))))*cos(x(3)))/l;

    dx = [dx(1);dx(2);dx(3);dx(4)].';
    
end


