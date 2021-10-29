function [z, out]=MyCost(x)
    
    global int_e
    int_e = 0;
    T0 = 0;Ts = 0.1;Tf = 10;
    t = T0:Ts:Tf;
    dt = Ts;
    
    num = (Tf-T0)/Ts;
    
    if length(x)==3   % PID Ctrlr
        Kp=100*x(1);
        Kd=10*x(2);
        Ki=100*x(3);
    else              % PD Ctrlr
        Kp=100*x(1);
        Kd=10*x(2);
        Ki=0;  
    end
    
    Gains = [Kp, Kd, Ki];
    desState = [0,0,0,0];
    oldState = [0,0,deg2rad(1),0];
    savedStates = [];
    
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
    Theta = savedStates(:,3);
    Theta_dot = savedStates(:,4);
    Theta_des = desState(3);
    Theta_dot_des = desState(4);
    
    % Cost Weights
    w1=2;
    w2=1;
     
    % Cost
    z = sum(w1*(Theta-Theta_des).^2 + w2*(Theta_dot-Theta_dot_des).^2);
    
    % OutPuts
    out.kp=Kp;
    out.ki=Ki;
    out.kd=Kd;
    out.t = t;
    out.u = u;
    out.States = savedStates;
    out.z=z;

end