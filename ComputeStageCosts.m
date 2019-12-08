function G = ComputeStageCosts( stateSpace, map )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
%%compute base index
ind_base=base_indx(map,BASE,stateSpace);
%%transition probability matrix
P=ComputeTransitionProbabilities( stateSpace, map);
%%find probability of which drone goes to base without crashing
base_neighbor=find_base_neighbor(stateSpace, map,BASE);
size_base_neighbor=size(base_neighbor,2);
P_NOT_CRASH=zeros(size_base_neighbor,5);

step1_coordinate=zeros(4);
step1_judgement=zeros(4);
step2_judgement=zeros(4);
step2_coordinate=zeros(4);
for i=1:size_base_neighbor
    %%first step(move by control)
    for j=1:5
        [step1_coordinate(j),step1_judgement(j)]=judge_coordinate(stateSpace,base_neighbor(i),j);
    %% second step(move by wind)
        if step1_judgement(j)==0
            for k=1:4
                [step2_coordinate(k),step2_judgement(k)]=judge_coordinate(stateSpace,step1_coordinate(j),k);
                if step1_coordinate(j)==ind_base
                    P_NOT_CRASH(i,j)=1-P_WIND;
                else
                    if step2_judgement(k)==0&&step2_coordinate(k)==ind_base
                        P_NOT_CRASH(i,j)=P_WIND/4*(1-step2_judgement(k));
                    end
                end
            end
        end
    end
end
%%initialize G
G=zeros(K,5);
for i=1:K
    for j=1:5
        if isempty(find(P(i,:,j)~=0, 1))
            G(i,j)=inf;
        else
            if any(base_neighbor==i)
                k=find(base_neighbor==i);
                G(i,j)=(P(i,ind_base,j)-P_NOT_CRASH(k,j))*Nc+(1-P(i,ind_base,j)+P_NOT_CRASH(k,j));
            else
                G(i,j)=P(i,ind_base,j)*Nc+(1-P(i,ind_base,j));
            end
        end
    end
end
G(TERMINAL_STATE_INDEX,:)=0;
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

function y=find_base_neighbor(stateSpace, map,BASE)
    ind_base=base_indx(map,BASE,stateSpace);
    m=1;
    for i=1:4
        [step1_coordinate(i),step1_judge(i)]=judge_coordinate(stateSpace,ind_base,i);
        if step1_judge(i)==0
            for j=1:5
                [step2_coordinate(i,j),step2_judge(i,j)]=judge_coordinate(stateSpace,step1_coordinate(i),j);
                if step2_judge(i,j)==0
                    base_neighbor(m)=step2_coordinate(i,j);
                    m=m+1;
                end
            end
        end
    end
    y=unique(base_neighbor);
end
