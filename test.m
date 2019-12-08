
P_1step(:,:,:)=zeros(K,K,5);
P_1step(:,:,5)=eye(K);
P_2step(:,:,:)=zeros(K,K,5);
P_3step=zeros(K);
P_4step=zeros(K);
for i=1:K
    %%first step
    for j=1:4
        jd_co(j,:)=judge_coordinate(stateSpace,i,j);
        coordinate(j)=jd_co(j,1);judgement(j)=jd_co(j,2);
        if judgement(j)==0
            P_1step(i,coordinate(j),j)=1;
        end
    end
    %{
    %%judgement of input north
    [orient_north,judgement_north]=judge_orient_north(stateSpace,i);
    if judgement_north==0
        P_1step(i,orient_north,1)=1;
    end
    %%judgement of input south
    [orient_south,judgement_south]=judge_orient_south(stateSpace,i);
    if judgement_south==0
        P_1step(i,orient_south,2)=1;
    end
    %%judgement of input east
    [orient_east,judgement_east]=judge_orient_east(stateSpace,i);
    if judgement_east==0
        P_1step(i,orient_east,3)=1;
    end
    %%judgement of input west
    [orient_west,judgement_west]=judge_orient_west(stateSpace,i);
    if judgement_west==0
        P_1step(i,orient_west,4)=1;
    end 
    
    %%second step
    %%find base index
    base_index=base_indx(map,BASE,stateSpace,i);
    %%compute second step probabilistic matrix
    if judgement_north==0
        [step2_orient_north,step2_judge_north]=judge_orient_north(stateSpace,orient_north);
        [step2_orient_south,step2_judge_south]=judge_orient_south(stateSpace,orient_north);
        [step2_orient_east,step2_judge_east]=judge_orient_east(stateSpace,orient_north);
        [step2_orient_west,step2_judge_west]=judge_orient_west(stateSpace,orient_north);
        P_2step(orient_north,orient_north,1)=1-P_WIND;
        P_2step(orient_north,step2_orient_north,1)=P_WIND/4*(1-step2_judge_north);
        P_2step(orient_north,step2_orient_south,1)=P_WIND/4*(1-step2_judge_south);
        P_2step(orient_north,step2_orient_east,1)=P_WIND/4*(1-step2_judge_east);
        P_2step(orient_north,step2_orient_west,1)=P_WIND/4*(1-step2_judge_west);
        P_2step(orient_north,base_index,1)=P_WIND/4*(step2_judge_north+step2_judge_south+step2_judge_east+step2_judge_west);
    end
        %}
end
P_r=P_1step(:,:,1)*P_2step(:,:,1);
p1=P_2step(:,:,1);
p2=P_1step(:,:,1);
P1=P(:,:,1);
    %{
    P_2step(i,i)=1-P_WIND;
    P_2step(i,north_xyphi_ind)=P_WIND/4*(1-judgement_north);
    P_2step(i,south_xyphi_ind)=P_WIND/4*(1-judgement_south);
    P_2step(i,east_xyphi_ind)=P_WIND/4*(1-judgement_east);
    P_2step(i,west_xyphi_ind)=P_WIND/4*(1-judgement_west);
    P_2step(i,ind_base)=P_WIND/4*(judgement_north+judgement_south+judgement_east+judgement_west);
    %}
%{
    %%Third step
    %%find shooter index
    [shooter_x,shooter_y]=find(map==SHOOTER);
    shooter_size=size(shooter_x,1);
    for j=1:shooter_size
        distance(j)=abs(shooter_x(j)-stateSpace(i,1))+abs(shooter_y(j)-stateSpace(i,2));
        if distance(j)<=R
            p_shoot(j)=GAMMA/(distance(j)+1);
        else
            p_shoot(j)=0;
        end
    end
    P_3step(i,i)=prod(1-p_shoot);
    P_3step(i,ind_base)=1-prod(1-p_shoot); 
    
    %%Fourth step
    [pick_up_x,pick_up_y]=find(map==PICK_UP);
    if(stateSpace(i,1)==pick_up_x&&stateSpace(i,2)==pick_up_y&&stateSpace(i,3)==0)
        P_4step(i,i+1)=1;
    else
        P_4step(i,i)=1;
    end

for i=1:5
    P_r(:,:,i)=P_4step*P_3step*P_2step*P_1step(:,:,i);
end
P_r1=P_r(:,:,1);
P1=P(:,:,1);
P_step=P_1step(:,:,1);
%}



