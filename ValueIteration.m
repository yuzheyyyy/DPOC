function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%%initialize V_0 and other parameters
V(:,1)=zeros(K-1,1);
P_shrink=zeros(K-1,K-1,5);
for j=1:5
    P_temp=P(:,:,j);
    P_temp(:,TERMINAL_STATE_INDEX)=[];P_temp(TERMINAL_STATE_INDEX,:)=[];
    P_shrink(:,:,j)=P_temp;
end
G_shrink=G;G_shrink(TERMINAL_STATE_INDEX,:)=[];
TOL=1e-4;
k=1;
while k>=1
    V_temp=zeros(K-1,5);
    for j=1:5
        V_temp(:,j)=G_shrink(:,j)+P_shrink(:,:,j)*V(:,k);
    end
    [V(:,k+1),ind(:,k+1)]=min(V_temp,[],2);
    if max(abs(V(:,k+1)-V(:,k)))<TOL
        break;
    end
    k=k+1;
end
J_opt=V(:,k+1);
J_opt=[J_opt(1:TERMINAL_STATE_INDEX-1);0;J_opt(TERMINAL_STATE_INDEX:end)];
u_opt_ind=ind(:,k+1);
u_opt_ind=[u_opt_ind(1:TERMINAL_STATE_INDEX-1);HOVER;u_opt_ind(TERMINAL_STATE_INDEX:end)];
end