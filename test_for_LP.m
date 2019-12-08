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
[x,fval] =linprog(f,LP_A,LP_b,[],[],lb);
for j=1:5
    G_temp2=G(:,j);
    G_temp2(TERMINAL_STATE_INDEX)=[];
    P_temp2=P(:,:,j);
    P_temp2(TERMINAL_STATE_INDEX,:)=[];
    P_temp2(:,TERMINAL_STATE_INDEX)=[];
    V(:,j)=G_temp2+P_temp2*x;
end
[value,ind]=min(V,[],2);

