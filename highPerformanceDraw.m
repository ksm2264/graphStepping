close all
clearvars

subj='JAC';

for walkNum=2
    
    %     if walkNum~=8&&walkNum~=9&&walkNum>4
    % path with the _pupilShadowMesh.mat files
    psmPath = '/media/karl/DATA/pupilShadowMesh/';
    % path with the .obj, .ply files (and .mat files containing the obj info (trimesh.mat) )
    meshPath = '/media/karl/DATA/allMeshes/';
    % path to save your KDtrees
    treePath = '/media/karl/DATA/trees3D/';
    
    
    
    load([psmPath subj '_' num2str(walkNum) '_pupilShadowMesh.mat']);
    
    load([meshPath subj '_' num2str(walkNum-1) '_out/triMesh.mat']);
    
    pc = pcread([meshPath subj '_' num2str(walkNum-1) '_out/texturedMesh.ply']);
    
    %% compute tri cens (calculate centers of triangles for NN color assignment)
    
    clear triCens
    % iterate over dimensions and calculate triangle center coordinate
    for dim = 1:3
        this_dim_vals = objStruct.v(objStruct.f',dim);
        this_dim_vals = reshape(this_dim_vals,[3 size(objStruct.f,1)]);
        triCens(:,dim) = mean(this_dim_vals,1)';
    end
    
    % check if you have a tree already, if not create onee
    if exist([treePath subj '_' num2str(walkNum) '.mat'])~=2
        kd_tree = KDTreeSearcher(pc.Location);
        save([treePath subj '_' num2str(walkNum) '.mat'],'kd_tree');
    else
        kd_tree=load([treePath subj '_' num2str(walkNum) '.mat']).kd_tree;
    end
    
    % assign colors to triangles based on nearest neighbor
    vertColors = pc.Color(knnsearch(kd_tree,objStruct.v),:);
    triColors = pc.Color(knnsearch(kd_tree,triCens),:);
    
    % rotate the obj data
    objStruct.v = objStruct.v*orig2alignedMat;
    
    % find closest triangle for each gaze intersection
    [kidx,D] = knnsearch(triCens*orig2alignedMat,gazeXYZ,'k',100);
    
    [ukidx,~,ic]=unique(kidx);
    
    ic = reshape(ic,size(kidx));
    gazeMeshColors = triColors(ukidx,:);
    gazeMeshTri = objStruct.f(ukidx,:);
    
    
    %%
    % setup triangle mesh graphic object
    figure(1)
    clf
    fh=trisurf(objStruct.f,objStruct.v(:,1),objStruct.v(:,2),objStruct.v(:,3));
    fh.FaceVertexCData =triColors;
    fh.FaceColor='flat';
    fh.EdgeColor='none';
    
    hold on
    
    % set up the burned triangles as a separate object for speed
    gmh = trisurf(gazeMeshTri,objStruct.v(:,1),objStruct.v(:,2),objStruct.v(:,3));
    gmh.FaceVertexCData = gazeMeshColors;
    gmh.FaceColor='flat';
    gmh.EdgeColor='none';
    gmh.FaceAlpha=0.5;
    
    burner = zeros(size(gazeMeshColors,1),1);
    
    
    
    axis equal
    hold on
    
    % pre plot all the steeps
    plot3(step_plantfoot_xyz(:,3),step_plantfoot_xyz(:,4),step_plantfoot_xyz(:,5),'m.','markersize',30);
    
    % initialize empty handles
    skelHandle = [];
    gazeVecHandle=[];
    
    drawBoxDim = 2.5;
    
    colormap jet
    cmap = colormap();
    
    max_D = 3*median(D(:));
    %%
    for idx = 1:length(shadow)
        
        this_loc = cens(idx,:);
        
        % if handles are empty, make them otherwise update them
        if isempty(skelHandle)
            skelHandle=skeletonDraw(shadow(idx,:,:));
        else
            updateSkeleton(skelHandle,shadow(idx,:,:));
        end
        
        if isempty(gazeVecHandle)
            gazeVecHandle = line([cens(idx,1) gazeXYZ(idx,1)],...
                [cens(idx,2) gazeXYZ(idx,2)],...
                [cens(idx,3) gazeXYZ(idx,3)],'linewidth',2,'color','b');
        else
            gazeVecHandle.XData = [cens(idx,1) gazeXYZ(idx,1)];
            gazeVecHandle.YData = [cens(idx,2) gazeXYZ(idx,2)];
            gazeVecHandle.ZData = [cens(idx,3) gazeXYZ(idx,3)];
        end
        
        % nice camera positioning
        campos(cens(idx,:) + [-1 1 0]);
        camtarget(cens(idx,:));
        camup([0 1 0]);
        camva(120)
        cf = gcf;
        cf.Position = [73 1 1848 961];
        
        %% comment this stuff if you don't want the mesh burning
%         burner(ic(idx,:)) = burner(ic(idx,:))  + (max_D-D(idx,:))';
%         burner(burner<0)=0;
%         burner_idx = min(max(round(burner(ic(idx,:))/(3*max_D)*256),1),256);
%         
%         gmh.FaceVertexCData(ic(idx,:),:) = uint8(256*cmap(burner_idx,:));
        %
        
        %%
        
        drawnow
        saveas(gcf,['/media/karl/DATA/visFrames/walk_vids/JAC/' num2str(walkNum) '/' num2str(idx) '.png']);
    end
    %     end
end