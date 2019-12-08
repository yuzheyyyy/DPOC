tic
P_step1(:,:,:)=zeros(K,K,5);
P_step2(:,:,:)=zeros(K,K+1,5);
P_step3(:,:,:)=zeros(K+1,K+1,5);
P_step4(:,:,:)=zeros(K+1,K,5);
P=zeros(K,K,5);
%%compute shooter index
[shooter_x,shooter_y]=find(map==SHOOTER);
shooter_size=size(shooter_x,1);
%%compute pick_up index
pick_up_ind=compute_pickup_index(stateSpace,map,PICK_UP);
%%first step
step1_coordinate=zeros(5);
step1_judgement=zeros(5);
for i=1:K
    for j=1:5
        [step1_coordinate(j),step1_judgement(j)]=judge_coordinate(stateSpace,i,j);
        if step1_judgement(j)==0
            P_step1(i,step1_coordinate(j),j)=1;
        end
    end
end
%%second step
step2_coordinate=zeros(4);
step2_judgement=zeros(4);
for i=1:K
    for j=1:5
        for k=1:4
            [step2_coordinate(k),step2_judgement(k)]=judge_coordinate(stateSpace,i,k);
            if step2_judgement(k)==0
                P_step2(i,step2_coordinate(k),j)=P_WIND/4*(1-step2_judgement(k));
            end
        end          
            P_step2(i,i,j)=1-P_WIND;
            P_step2(i,K+1,j)=P_WIND/4*sum(step2_judgement);
    end
end
%%third step
distance_x=zeros(K,shooter_size,5);
distance_y=zeros(K,shooter_size,5);
distance=zeros(K,shooter_size,5);
p_shoot=zeros(K,shooter_size,5);
for i=1:K
    for j=1:5
        for m=1:shooter_size
            distance_x(i,m,j)=abs(shooter_x(m)-stateSpace(i,1));
            distance_y(i,m,j)=abs(shooter_y(m)-stateSpace(i,2));
            distance(i,m,j)=distance_x(i,m,j)+distance_y(i,m,j);
            if distance(i,m,j)<=R
                p_shoot(i,m,j)=GAMMA/(distance(i,m,j)+1);
            else
                p_shoot(i,m,j)=0;
            end
        end
        P_step3(i,i,j)=prod(1-p_shoot(i,:,j));
        P_step3(i,K+1,j)=1-prod(1-p_shoot(i,:,j));
    end
end
for j=1:5
    P_step3(K+1,K+1,j)=1;
end


%%fourth step
for i=1:K
    for j=1:5
        if i==pick_up_ind
            P_step4(i,i+1,j)=1;
        else
            P_step4(i,i,j)=1;
        end
    end
end 

for j=1:5
    P_step4(K+1,base_indx(map,BASE,stateSpace),j)=1;
    P(:,:,j)=P_step1(:,:,j)*P_step2(:,:,j)*P_step3(:,:,j)*P_step4(:,:,j);
    P(TERMINAL_STATE_INDEX,:,j)=0;P(TERMINAL_STATE_INDEX,TERMINAL_STATE_INDEX,j)=1;
end  
t2=toc



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
