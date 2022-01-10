function [foot_graph,D_graph,hod_graph,dot_graph,momentum_graph] = getLargeCutoffGraph(pc_nodes,goalLoc,stepLength,dot_cutoff,hod_cutoff)

if nargin==3
    silenceVec=[0 0 0];
end

stats=cleanStats(load('stats.mat').stats);

dot_cutoff = min(stats.step_dot);
hod_cutoff =max(stats.step_hod);

%% first we do hard cutoffs (within the ranges of 3 constraints)

% lower and upper bound cutoff for step distance
[within_range,D_raw] = rangesearch(pc_nodes.Location,pc_nodes.Location,stepLength*max(stats.step_dist));
D = cellfun(@(x,y) y(x>stepLength*min(stats.step_dist)),D_raw,D_raw,'UniformOutput',false);
within_range = cellfun(@(x,y) y(x>stepLength*min(stats.step_dist)),D_raw,within_range,'UniformOutput',false);

% lower and upper bound for direction vector dot product, and upper bound for hod
within_range_vecs = cellfun(@(x,y) normr(pc_nodes.Location(x,[1 3]) - y),...
    within_range,num2cell(pc_nodes.Location(:,[1 3]),2),'UniformOutput',false);

node_goal_dir = cellfun(@(y) normr(goalLoc([1 3]) - y),...
    num2cell(pc_nodes.Location(:,[1 3]),2),'UniformOutput',false);

within_range_dot = cellfun(@(x,y) sum(x.*y,2),within_range_vecs,node_goal_dir,'UniformOutput',false);


within_range_hod = cellfun(@(x,y,z) abs(pc_nodes.Location(x,2)' - y)./vecnorm(pc_nodes.Location(x,[1 3])-z,2,2)',...
    within_range,num2cell(pc_nodes.Location(:,2)),num2cell(pc_nodes.Location(:,[1 3]),2),...
    'UniformOutput',false);

connect_these = cellfun(@(x,y,z) y(x>=dot_cutoff&z'<=hod_cutoff),within_range_dot,within_range,within_range_hod,'UniformOutput',false);
connected_D = cellfun(@(x,y,z) y(x>=dot_cutoff&z'<=hod_cutoff),within_range_dot,D,within_range_hod,'UniformOutput',false);
connected_dot = cellfun(@(x,z) x(x>=dot_cutoff&z'<=hod_cutoff),within_range_dot,within_range_hod,'UniformOutput',false);
connected_hod = cellfun(@(x,z) z(x>=dot_cutoff&z'<=hod_cutoff),within_range_dot,within_range_hod,'UniformOutput',false);

%% connect based on within bounds


lindex = cellfun(@(x,y) sub2ind(pc_nodes.Count*ones(1,2),x*ones(size(y)),y),...
    num2cell((1:pc_nodes.Count)'),connect_these,'UniformOutput',false);
%
lindex_all = horzcat(lindex{:});
connection_strengths = ones(size(lindex_all));
[i_index,j_index] = ind2sub([pc_nodes.Count pc_nodes.Count],lindex_all);
G = sparse(i_index,j_index,double(connection_strengths),pc_nodes.Count,pc_nodes.Count);
foot_graph = digraph(G);

connection_strengths = horzcat(connected_D{:});
G = sparse(i_index,j_index,double(connection_strengths),pc_nodes.Count,pc_nodes.Count);
D_graph = digraph(G);

connection_strengths = horzcat(connected_hod{:});
connection_strengths(connection_strengths==0)=1e-3;
G = sparse(i_index,j_index,double(connection_strengths),pc_nodes.Count,pc_nodes.Count);
hod_graph = digraph(G);

connection_strengths = vertcat(connected_dot{:});
G = sparse(i_index,j_index,double(connection_strengths),pc_nodes.Count,pc_nodes.Count);
dot_graph = digraph(G);

end


