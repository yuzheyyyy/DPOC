function [y,z]=judge_orient_west(stateSpace,x)
    west_x_ind=find(stateSpace(:,1)==stateSpace(x,1)-1);
    west_y_ind=find(stateSpace(:,2)==stateSpace(x,2));
    west_phi_ind=find(stateSpace(:,3)==stateSpace(x,3));
    west_xy_ind=intersect(west_x_ind,west_y_ind);
    west_xyphi_ind=intersect(west_xy_ind,west_phi_ind);
    y=west_xyphi_ind;
    z=isempty(west_xyphi_ind);
end  