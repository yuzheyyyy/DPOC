global TERMINAL_STATE_INDEX
%%compute base index
ind_base=base_indx(map,BASE,stateSpace);
%%transition probability matrix
P=ComputeTransitionProbabilities( stateSpace, map);
%%find probability of which drone goes to base without crashing
base_neighbor=find_base_neighbor(stateSpace, map,BASE);
size_base_neighbor=size(base_neighbor,2);
P_NOT_CRASH=zeros(size_base_neighbor,5);
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
G1=zeros(K,5);
for i=1:K
    for j=1:5
        if isempty(find(P(i,:,j)~=0))
            G1(i,j)=inf;
        else
            if any(base_neighbor==i)
                k=find(base_neighbor==i);
                G1(i,j)=(P(i,ind_base,j)-P_NOT_CRASH(k,j))*Nc+(1-P(i,ind_base,j)+P_NOT_CRASH(k,j));
            else
                G1(i,j)=P(i,ind_base,j)*Nc+(1-P(i,ind_base,j));
            end
        end
    end
end
G1(TERMINAL_STATE_INDEX,:)=0;




