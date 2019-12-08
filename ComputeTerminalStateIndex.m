function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

global DROP_OFF
    [delivery_station_x,delivery_station_y]=find(map==DROP_OFF);
    ind_x=find(stateSpace(:,1)==delivery_station_x);
    ind_y=find(stateSpace(:,2)==delivery_station_y);
    ind_status=find(stateSpace(:,3)==1);
    ind_xy=intersect(ind_x,ind_y);
    ind_xy_status=intersect(ind_xy,ind_status);
    stateIndex=ind_xy_status;
end
