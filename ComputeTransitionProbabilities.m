function P = ComputeTransitionProbabilities( stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

P_step1(:,:,:)=zeros(K,K,5);
P_step2(:,:,:)=zeros(K,K+1,5);
P_step3(:,:,:)=zeros(K+1,K+1,5);
P_step4(:,:,:)=zeros(K+1,K,5);
%%compute shooter index
[shooter_x,shooter_y]=find(map==SHOOTER);
shooter_size=size(shooter_x,1);
%%compute pick_up index
pick_up_ind=compute_pickup_index(stateSpace,map,PICK_UP);
%%initialization
step1_coordinate=zeros(5);
step1_judgement=zeros(5);
step2_coordinate=zeros(4);
distance_x=zeros(K,shooter_size,5);
distance_y=zeros(K,shooter_size,5);
distance=zeros(K,shooter_size,5);
p_shoot=zeros(K,shooter_size,5);
for i=1:K
    %%first step
    for j=1:5
        [step1_coordinate(j),step1_judgement(j)]=judge_coordinate(stateSpace,i,j);
        if step1_judgement(j)==0
            P_step1(i,step1_coordinate(j),j)=1;
        end
    end
    %%second step
    for j=1:5
        if step1_judgement(j)==0
            for k=1:4
                [step2_coordinate(k),step2_judgement(k)]=judge_coordinate(stateSpace,step1_coordinate(j),k);
                if step2_judgement(k)==0
                    P_step2(step1_coordinate(j),step2_coordinate(k),j)=P_WIND/4*(1-step2_judgement(k));
                end
            end          
                P_step2(step1_coordinate(j),step1_coordinate(j),j)=1-P_WIND;
                P_step2(step1_coordinate(j),K+1,j)=P_WIND/4*sum(step2_judgement);
        end
    end 
    %%third step
    for j=1:5
        if step1_judgement(j)==0
            temp=find(P_step2(step1_coordinate(j),:,j));
            delete_ind=find(temp>=K+1);temp(delete_ind)=[];
            cell{j}=temp;size_cell(j)=size(cell{j},2);
            for k=1:size_cell(j)
                for m=1:shooter_size
                    distance_x(k,m,j)=abs(shooter_x(m)-stateSpace(cell{j}(k),1));
                    distance_y(k,m,j)=abs(shooter_y(m)-stateSpace(cell{j}(k),2));
                    distance(k,m,j)=distance_x(k,m,j)+distance_y(k,m,j);
                    if distance(k,m,j)<=R
                        p_shoot(k,m,j)=GAMMA/(distance(k,m,j)+1);
                    else
                        p_shoot(k,m,j)=0;
                    end
                end
                P_step3(cell{j}(k),cell{j}(k),j)=prod(1-p_shoot(k,:,j));
                P_step3(cell{j}(k),K+1,j)=1-prod(1-p_shoot(k,:,j));
            end
        end
        P_step3(K+1,K+1,j)=1;
    end
    %%fourth step
    for j=1:5
        if step1_judgement(j)==0
        %%compute where second step goes to
            temp=find(P_step2(step1_coordinate(j),:,j));
            delete_ind=find(temp>=K+1);temp(delete_ind)=[];
            cell{j}=temp;size_cell(j)=size(cell{j},2);
            for k=1:size_cell(j)
                if cell{j}(k)==pick_up_ind
                    P_step4(cell{j}(k),cell{j}(k)+1,j)=1;
                else
                    P_step4(cell{j}(k),cell{j}(k),j)=1;
                end
            end
            P_step4(K+1,base_indx(map,BASE,stateSpace),j)=1;
        end 
    end
end
for j=1:5
    P(:,:,j)=P_step1(:,:,j)*P_step2(:,:,j)*P_step3(:,:,j)*P_step4(:,:,j);
    P(TERMINAL_STATE_INDEX,:,j)=0;P(TERMINAL_STATE_INDEX,TERMINAL_STATE_INDEX,j)=1;
end  
end


function [y,z]=judge_coordinate(stateSpace,x,p)
switch p
    case 1
        north_x_ind=find(stateSpace(:,1)==stateSpace(x,1));
        north_y_ind=find(stateSpace(:,2)==stateSpace(x,2)+1);
        north_phi_ind=find(stateSpace(:,3)==stateSpace(x,3));
        north_xy_ind=intersect(north_x_ind,north_y_ind);
        north_xyphi_ind=intersect(north_xy_ind,north_phi_ind);
        y=north_xyphi_ind;
        z=isempty(north_xyphi_ind);
        if z==1
            y=0;
        end
    case 2
        south_x_ind=find(stateSpace(:,1)==stateSpace(x,1));
        south_y_ind=find(stateSpace(:,2)==stateSpace(x,2)-1);
        south_phi_ind=find(stateSpace(:,3)==stateSpace(x,3));
        south_xy_ind=intersect(south_x_ind,south_y_ind);
        south_xyphi_ind=intersect(south_xy_ind,south_phi_ind);
        y=south_xyphi_ind;
        z=isempty(south_xyphi_ind);
        if z==1
            y=0;
        end
    case 3
        east_x_ind=find(stateSpace(:,1)==stateSpace(x,1)+1);
        east_y_ind=find(stateSpace(:,2)==stateSpace(x,2));
        east_phi_ind=find(stateSpace(:,3)==stateSpace(x,3));
        east_xy_ind=intersect(east_x_ind,east_y_ind);
        east_xyphi_ind=intersect(east_xy_ind,east_phi_ind);
        y=east_xyphi_ind;
        z=isempty(east_xyphi_ind);
        if z==1
            y=0;
        end
    case 4
        west_x_ind=find(stateSpace(:,1)==stateSpace(x,1)-1);
        west_y_ind=find(stateSpace(:,2)==stateSpace(x,2));
        west_phi_ind=find(stateSpace(:,3)==stateSpace(x,3));
        west_xy_ind=intersect(west_x_ind,west_y_ind);
        west_xyphi_ind=intersect(west_xy_ind,west_phi_ind);
        y=west_xyphi_ind;
        z=isempty(west_xyphi_ind);
        if z==1
            y=0;
        end
    case 5
        y=x;
        z=0;
end       
end

function y=base_indx(map,BASE,stateSpace)
    [base_x,base_y]=find(map==BASE);
    ind_x=find(stateSpace(:,1)==base_x);
    ind_y=find(stateSpace(:,2)==base_y);
    ind_status=find(stateSpace(:,3)==0);
    ind_xy=intersect(ind_x,ind_y);
    y=intersect(ind_xy,ind_status);
end
function y=compute_pickup_index(stateSpace,map,PICK_UP)
    [base_x,base_y]=find(map==PICK_UP);
    ind_x=find(stateSpace(:,1)==base_x);
    ind_y=find(stateSpace(:,2)==base_y);
    ind_status=find(stateSpace(:,3)==0);
    ind_xy=intersect(ind_x,ind_y);
    y=intersect(ind_xy,ind_status);
end
