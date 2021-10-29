clc;clear;close all

Kp = 64;
Kd = 5;
Ki = 0;
Gains = [Kp,Kd,Ki];

global Ts int_e
int_e = 0;
T0 = 0;Ts = 0.01; Tf = 20;
t = T0:Ts:Tf;
num = (Tf-T0)/Ts;
x0  = [0,0,deg2rad(1),0];
oldState = x0;
savedStates = [];
desState = [0,0,0,0];

dt = Ts;

for i = 1:num
    
    u = PID_Ctrlr(oldState, desState, Gains);
    k1 = Inv_Pdlm(oldState, u);
    k2 = Inv_Pdlm(oldState+dt/2*k1, u);
    k3 = Inv_Pdlm(oldState+dt/2*k2, u);
    k4 = Inv_Pdlm(oldState+dt*k3, u);

    newState = oldState + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    % All states wrapped to 2pi
    if newState(3) > pi
        newState(3) = newState(3) - 2*pi;
    elseif newState(3) < -pi
        newState(3) = newState(3) + 2*pi;
    end
    
    savedStates = [savedStates;oldState];
    oldState = newState;
    
end
figure;
plot(t(1:end-1), rad2deg(savedStates(:,3)), 'k-.','LineWidth', 2);hold on 
plot(t(1:end-1), rad2deg(savedStates(:,4)), 'b-.','LineWidth', 2);grid on
xlabel('Time'); ylabel('Outputs'); title('PID Responce')
legend('\theta','\theta_d_o_t')

