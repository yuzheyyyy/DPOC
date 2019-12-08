function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
%%initialize policy and parameters
policy(:,1)=ones(K-1,1);
P_shrink=zeros(K-1,K-1,5);
for j=1:5
    P_temp=P(:,:,j);
    P_temp(:,TERMINAL_STATE_INDEX)=[];P_temp(TERMINAL_STATE_INDEX,:)=[];
    P_shrink(:,:,j)=P_temp;
end
G_shrink=G;G_shrink(TERMINAL_STATE_INDEX,:)=[];
%%initialize first J_0
J(:,1)=(eye(K-1)-P_shrink(:,:,5))\G_shrink(:,5);
k=1;TOL=1e-4;
while k>=1
    for j=1:5
        J_temp(:,j)=G_shrink(:,j)+P_shrink(:,:,j)*J(:,k);
    end
    [J_temp2,ind]=min(J_temp,[],2);
    transition_matrix=[];stage_cost=[];
    for i=1:K-1
        transition_matrix=[transition_matrix;P_shrink(i,:,ind(i))];
        stage_cost=[stage_cost;G_shrink(i,ind(i))];
    end
    policy(:,k+1)=ind;
    J(:,k+1)=(eye(K-1)-transition_matrix)\stage_cost;
    if max(abs(J(:,k+1)-J(:,k)))<TOL
        break;
    end
    k=k+1;
end
J_opt=J(:,k+1);
J_opt=[J_opt(1:TERMINAL_STATE_INDEX-1);0;J_opt(TERMINAL_STATE_INDEX:end)];
u_opt_ind=ind;
u_opt_ind=[u_opt_ind(1:TERMINAL_STATE_INDEX-1);HOVER;u_opt_ind(TERMINAL_STATE_INDEX:end)];
end
