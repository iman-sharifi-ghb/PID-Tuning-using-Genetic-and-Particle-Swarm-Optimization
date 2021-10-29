clc;
clear;
close all;

%% Problem Definition

CostFunction=@(x) MyCost(x);       % Cost Function

nVar=2;                     % Number of Variables
% 2 for PD Ctrlr $ 3 for PID Ctrlr

VarSize=[1 nVar];           % Size of Variables Matrix

VarMin=-1;                  % Lower Bound of Variables
VarMax= 1;                  % Upper Bound of Variables

VarRange=[VarMin VarMax];   % Variation Range of Variables


%% GA Parameters

MaxIt=50;      % Maximum Number of Iterations

nPop=100;        % Population Size

pCrossover=0.7;                         % Crossover Percentage
nCrossover=round(pCrossover*nPop/2)*2;  % Number of Parents (Offsprings)

pMutation=0.2;                      % Mutation Percentage
nMutation=round(pMutation*nPop);    % Number of Mutants


%% Initialization

% Empty Structure to Hold Individuals Data
empty_individual.Position=[];
empty_individual.Cost=[];
empty_individual.Out=[];

% Create Population Matrix
pop=repmat(empty_individual,nPop,1);

% Initialize Positions
for i=1:nPop
    pop(i).Position=unifrnd(VarMin,VarMax,VarSize);
    [pop(i).Cost, pop(i).Out]=CostFunction(pop(i).Position);
end

% Sort Population
pop=SortPopulation(pop);

% Store Best Solution
BestSol=pop(1);

% Vector to Hold Best Cost Values
BestCost=zeros(MaxIt,1);

%% GA Main Loop

for it=1:MaxIt
    
    % Crossover
    popc=repmat(empty_individual,nCrossover/2,2);
    for k=1:nCrossover/2
        
        i1=randi([1 nPop]);
        i2=randi([1 nPop]);
        
        p1=pop(i1);
        p2=pop(i2);
        
        [popc(k,1).Position, popc(k,2).Position]=Crossover(p1.Position,p2.Position,VarRange);
        
        [popc(k,1).Cost, popc(k,1).Out]=CostFunction(popc(k,1).Position);
        [popc(k,2).Cost, popc(k,2).Out]=CostFunction(popc(k,2).Position);
        
    end
    popc=popc(:);
    
    
    % Mutation
    popm=repmat(empty_individual,nMutation,1);
    for k=1:nMutation
        
        i=randi([1 nPop]);
        
        p=pop(i);
        
        popm(k).Position=Mutate(p.Position,VarRange);
        
        [popm(k).Cost, popm(k).Out]=CostFunction(popm(k).Position);
        
    end
    
    % Merge Population
    pop=[pop
         popc
         popm];
    
    % Sort Population
    pop=SortPopulation(pop);
    
    % Delete Extra Individuals
    pop=pop(1:nPop);
    
    % Update Best Solution
    BestSol=pop(1);
    
    % Store Best Cost
    BestCost(it)=BestSol.Cost;
    
    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
    
    % Plot Step Response
    t     = BestSol.Out.t;
    theta = BestSol.Out.States(:,3);
    theta_dot = BestSol.Out.States(:,4);
    
    figure(1);
    subplot(121);
    plot(t(1:end-1).', rad2deg(theta), 'LineWidth', 2);hold on;
    plot(t(1:end-1).', rad2deg(theta_dot), 'LineWidth', 2);
    grid on; title('PID Tuning using GA'); 
    xlabel('Time');ylabel('Outputs');legend('\theta','\theta_d_o_t')
    hold off;
    
    subplot(122);
    plot(it, BestSol.Out.kp,'bs', 'LineWidth', 2);hold on;
    plot(it, BestSol.Out.kd,'ro', 'LineWidth', 2);hold on;
    plot(it, BestSol.Out.ki,'g*', 'LineWidth', 2);
    grid on; title('PID Tuning using GA'); 
    xlabel('Time');ylabel('Outputs');legend('K_p','K_d','K_i')
%     hold off;
    
end

%% Plots

figure;
plot(BestCost, 'LineWidth', 2);grid on
xlabel('Iteration'),ylabel('Cost')
title('Best of Costs')

