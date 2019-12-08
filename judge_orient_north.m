function [y,z]=judge_orient_north(stateSpace,x)
    north_x_ind=find(stateSpace(:,1)==stateSpace(x,1));
    north_y_ind=find(stateSpace(:,2)==stateSpace(x,2)+1);
    north_phi_ind=find(stateSpace(:,3)==stateSpace(x,3));
    north_xy_ind=intersect(north_x_ind,north_y_ind);
    north_xyphi_ind=intersect(north_xy_ind,north_phi_ind);
    y=north_xyphi_ind;
    z=isempty(north_xyphi_ind);
end