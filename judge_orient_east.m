function [y,z]=judge_orient_east(stateSpace,x)
    east_x_ind=find(stateSpace(:,1)==stateSpace(x,1)+1);
    east_y_ind=find(stateSpace(:,2)==stateSpace(x,2));
    east_phi_ind=find(stateSpace(:,3)==stateSpace(x,3));
    east_xy_ind=intersect(east_x_ind,east_y_ind);
    east_xyphi_ind=intersect(east_xy_ind,east_phi_ind);
    y=east_xyphi_ind;
    z=isempty(east_xyphi_ind);
end