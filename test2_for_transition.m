tic
P_step1(:,:,:)=zeros(K,K,5);
P_step2(:,:,:)=zeros(K,K+1,5);
P_step3(:,:,:)=zeros(K+1,K+1,5);
P_step4(:,:,:)=zeros(K+1,K,5);
%%compute shooter index
[shooter_x,shooter_y]=find(map==SHOOTER);
shooter_size=size(shooter_x,1);
%%compute base index
ind_base=base_indx(map,BASE,stateSpace);
%%compute terminal state
terminal_state=ComputeTerminalStateIndex(stateSpace,map);
%%compute pick_up index
pick_up_ind=compute_pickup_index(stateSpace,map,PICK_UP);
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
            P_step4(K+1,ind_base,j)=1;
        end 
    end
end
for j=1:5
    P(:,:,j)=P_step1(:,:,j)*P_step2(:,:,j)*P_step3(:,:,j)*P_step4(:,:,j);
    P(terminal_state,:,j)=0;P(terminal_state,terminal_state,j)=1;
end
%{
P1=P(:,:,1);
P2=P_step3(:,:,1);
t2=toc
%}