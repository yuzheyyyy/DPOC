%%initialize V_0 and other parameters
V(:,1)=zeros(K-1,1);
P_shrink=zeros(K-1,K-1,5);
for j=1:5
    P_temp=P(:,:,j);
    P_temp(:,TERMINAL_STATE_INDEX)=[];P_temp(TERMINAL_STATE_INDEX,:)=[];
    P_shrink(:,:,j)=P_temp;
end
G_shrink=G;G_shrink(TERMINAL_STATE_INDEX,:)=[];
TOL=1e-3;
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