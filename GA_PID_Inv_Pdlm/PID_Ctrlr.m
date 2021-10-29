function u = PID_Ctrlr(x, xd, Gains)
global int_e
e = x(3) - xd(3);
e_dot = x(4) - xd(4);
int_e = int_e + e;

Kp = Gains(1);Kd = Gains(2);Ki = Gains(3);
u = Kp*e + Kd*e_dot + Ki*int_e;

end

