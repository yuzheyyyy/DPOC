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
k=1;TOL=1e-3;
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
%{
P_shrink_ini=P(:,:,5);G_shrink_ini=G(:,5);
P_shrink_ini(TERMINAL_STATE_INDEX,:)=[];P_shrink_ini(:,TERMINAL_STATE_INDEX)=[];
G_shrink_ini(TERMINAL_STATE_INDEX,:)=[];
%%initialize first J_0
J(:,1)=(eye(K-1)-P_shrink_ini)\G_shrink_ini;
%%initialize second miu
for j=1:5
    P_shrink=P(:,:,j);G_shrink=G(:,j);
    P_shrink(TERMINAL_STATE_INDEX,:)=[];P_shrink(:,TERMINAL_STATE_INDEX)=[];
    G_shrink(TERMINAL_STATE_INDEX,:)=[];
    J_temp_ini(:,j)=G_shrink+P_shrink*J(:,1);
end
[J_temp_ini2,ind_ini]=min(J_temp_ini,[],2);
for i=1:K
    if i==TERMINAL_STATE_INDEX
        continue;
    end
    transition_matrix=[];stage_cost=[];
    transition_matrix=[transition_matrix;P(i,:,ind_ini(i))];
    stage_cost=[stage_cost;G(i,ind_ini(i))];
end


TOL=1e-3;
k=1;
while k>=1

    for i=1:K-1
        [J_temp2,ind]=min(J_temp,[],2);
        
end
%}