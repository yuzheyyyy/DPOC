function [y,z]=judge_orient_south(stateSpace,x)
    south_x_ind=find(stateSpace(:,1)==stateSpace(x,1));
    south_y_ind=find(stateSpace(:,2)==stateSpace(x,2)-1);
    south_phi_ind=find(stateSpace(:,3)==stateSpace(x,3));
    south_xy_ind=intersect(south_x_ind,south_y_ind);
    south_xyphi_ind=intersect(south_xy_ind,south_phi_ind);
    y=south_xyphi_ind;
    z=isempty(south_xyphi_ind);
end