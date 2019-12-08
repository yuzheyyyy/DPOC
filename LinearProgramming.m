function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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
f=-ones(K-1,1);LP_A=[];LP_b=[];lb=zeros(K-1,1);
for j=1:5
    ind=find(G(:,j)==inf);
    P_temp=P(:,:,j);
    P_temp(:,TERMINAL_STATE_INDEX)=[];
    P_temp([ind;TERMINAL_STATE_INDEX],:)=[];
    G_temp=G(:,j);
    G_temp([ind;TERMINAL_STATE_INDEX])=[];
    eye_temp=eye(K);
    eye_temp([ind;TERMINAL_STATE_INDEX],:)=[];
    eye_temp(:,TERMINAL_STATE_INDEX)=[];
    LP_A=[LP_A;eye_temp-P_temp];
    LP_b=[LP_b;G_temp];
end
value =linprog(f,LP_A,LP_b,[],[],lb);
for j=1:5
    G_temp2=G(:,j);
    G_temp2(TERMINAL_STATE_INDEX)=[];
    P_temp2=P(:,:,j);
    P_temp2(TERMINAL_STATE_INDEX,:)=[];
    P_temp2(:,TERMINAL_STATE_INDEX)=[];
    V(:,j)=G_temp2+P_temp2*value;
end
[value,ind]=min(V,[],2);
J_opt=value;
J_opt=[J_opt(1:TERMINAL_STATE_INDEX-1);0;J_opt(TERMINAL_STATE_INDEX:end)];
u_opt_ind=ind;
u_opt_ind=[u_opt_ind(1:TERMINAL_STATE_INDEX-1);HOVER;u_opt_ind(TERMINAL_STATE_INDEX:end)];
end

