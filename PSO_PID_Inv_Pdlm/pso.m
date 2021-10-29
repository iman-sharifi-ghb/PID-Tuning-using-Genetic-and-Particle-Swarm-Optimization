clc;
clear;
close all;

%% Problem Definition

CostFunction=@(x) MyCost(x);       % Cost Function

nVar=3;                     % Number of Variables

VarSize=[1 nVar];           % Size of Variables Matrix

VarMin=-1;                  % Lower Bound of Variables
VarMax= 1;                  % Upper Bound of Variables

VarRange=[VarMin VarMax];   % Variation Range of Variables

VelMax=(VarMax-VarMin)/10;  % Maximum Velocity
VelMin=-VelMax;             % Minimum Velocity

%% PSO Parameters

MaxIt=50;          % Maximum Number of Iterations

nPop=50;            % Swarm (Population) Size

% Definition of Constriction Coefficients
phi1=2.05;
phi2=2.05;
phi=phi1+phi2;
chi=2/(phi-2+sqrt(phi^2-4*phi));

w=chi;
c1=phi1*chi;
c2=phi2*chi;

%% Initialization

% Empty Structure to Hold Individuals Data
empty_individual.Position=[];
empty_individual.Velocity=[];
empty_individual.Cost=[];
empty_individual.Out=[];
empty_individual.Best.Position=[];
empty_individual.Best.Cost=[];
empty_individual.Best.Out=[];

% Create Population Matrix
pop=repmat(empty_individual,nPop,1);

% Global Best
BestSol.Cost=inf;

% Initialize Positions
for i=1:nPop
    
    pop(i).Position=unifrnd(VarMin,VarMax,VarSize);
    pop(i).Velocity=zeros(VarSize);
    
    [pop(i).Cost, pop(i).Out]=CostFunction(pop(i).Position);
    
    pop(i).Best.Position=pop(i).Position;
    pop(i).Best.Cost=pop(i).Cost;
    pop(i).Best.Out=pop(i).Out;
    
    if pop(i).Best.Cost<BestSol.Cost
        BestSol=pop(i).Best;
    end
    
end

% Vector to Hold Best Cost Values
BestCost=zeros(MaxIt,1);

%% PSO Main Loop

for it=1:MaxIt
    
    for i=1:nPop
        
        % Update Velocity
        pop(i).Velocity=w*pop(i).Velocity ...
            + c1*rand(VarSize).*(pop(i).Best.Position-pop(i).Position) ...
            + c2*rand(VarSize).*(BestSol.Position-pop(i).Position);
        
        % Apply Velocity Bounds
        pop(i).Velocity=min(max(pop(i).Velocity,VelMin),VelMax);
        
        % Update Position
        pop(i).Position=pop(i).Position+pop(i).Velocity;
        
        % Velocity Reflection
        flag=(pop(i).Position<VarMin | pop(i).Position>VarMax);
        pop(i).Velocity(flag)=-pop(i).Velocity(flag);
        
        % Apply Position Bounds
        pop(i).Position=min(max(pop(i).Position,VarMin),VarMax);
        
        % Evaluation
        [pop(i).Cost, pop(i).Out]=CostFunction(pop(i).Position);
        
        % Update Personal Best
        if pop(i).Cost<pop(i).Best.Cost
            
            pop(i).Best.Position=pop(i).Position;
            pop(i).Best.Cost=pop(i).Cost;
            pop(i).Best.Out=pop(i).Out;
            
            % Update Global Best
            if pop(i).Best.Cost<BestSol.Cost
                BestSol=pop(i).Best;
            end
            
        end
        
    end
    
    % Store Best Cost
    BestCost(it)=BestSol.Cost;
    
    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
    
    % Plot Step Response
    figure(1);
    subplot(121);
    t     = BestSol.Out.t;
    theta = BestSol.Out.States(:,3);
    theta_dot = BestSol.Out.States(:,4);
    plot(t(1:end-1).', rad2deg(theta), 'LineWidth', 2);hold on;
    plot(t(1:end-1).', rad2deg(theta_dot), 'LineWidth', 2);
    grid on; title('PID Tuning using PSO'); 
    xlabel('Time');ylabel('Outputs');legend('\theta','\theta_d_o_t')
    hold off;
    
    subplot(122);
    plot(it, BestSol.Out.kp,'bs', 'LineWidth', 2);hold on;
    plot(it, BestSol.Out.kd,'ro', 'LineWidth', 2);hold on;
    plot(it, BestSol.Out.ki,'g*', 'LineWidth', 2);
    grid on; title('PID Tuning using PSO'); 
    xlabel('Time');ylabel('Outputs');legend('K_p','K_d','K_i')
    
end

%% Plots

figure;
plot(BestCost, 'LineWidth', 2);grid on
xlabel('Iteration'),ylabel('Cost')
title('Best of Costs')

%% Rerun Program
% if BestCost(end)>10
%     pso
% end

