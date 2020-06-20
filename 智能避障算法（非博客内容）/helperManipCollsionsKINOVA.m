classdef helperManipCollsionsKINOVA < robotics.manip.internal.InternalAccess
    % This file is for internal use only and may be modified or removed in
    % a future release.
    %
    %exampleHelperManipCollsions Tools for manipulator collisions
    %   This example helper has tools for collision checking. The object is
    %   constructed as a collision tree using either primitives or meshes.
    %   Helper methods are used for self-collisions and environmental
    %   collisions.
    %
    %   OBJ = exampleHelperManipCollsions(TREE) creates a collision tree
    %   using the visual meshes in TREE, the associated RigidBodyTree
    %   object, as the collision meshes.
    %
    %   OBJ = exampleHelperManipCollsions(TREE, PRIMITIVESIZES) creates a
    %   collision tree using primitives based on the size of each link.
    %   Here, PRIMITVESIZES is an (N+1)x1 cell array for the base body and
    %   the N bodies in the associated rigidBodyTree object, where each
    %   element is a 1x3 cell array of the form {RAD LEN TFORM}. RAD and
    %   LEN are the radius and length of the associated collisionCylinder
    %   collision object, and TFORM is the transform of the collision
    %   object's origin with respect to the associated joint frame.
    %
    %   OBJ = exampleHelperManipCollsions(URDFPATH, MESHPATH) creates a
    %   collision tree using the URDF and collision meshes defined by the
    %   URDF at location URDFPATH with meshes at location MESHPATH.
    %
    %   OBJ = exampleHelperManipCollsions(___, 'PropertyName', PropertyValue, ..) 
    %   sets additional properties specified as name-value pairs. You can
    %   specify multiple properties in any order.
    %
    %   EXAMPLEHELPERMANIPCOLLISIONS Properties:
    %      ExhaustiveChecking   - Boolean that indicates when to stop checking
    %
    %   EXAMPLEHELPERMANIPCOLLISIONS Methods:
    %      checkRobotSelfCollision    - Check collisions between robot bodies
    %      checkRobotWorldCollision   - Check collisions between robot and other collision meshes
    %      showCollision              - Plot collision results
    %      showCollisionTree          - Plot rigid body tree uses collision objects as rigid body visuals
    %      showCollisionObjects       - Plot collision objects using pose from rigidbodytree configuration
    %
    %   Example using the tree's visual meshes and self-collision:
    %      % Load the Kinova Gen3 manipulator
    %      robot = loadrobot('kinovaGen3', 'DataFormat', 'column');
    %
    %      % Create a collision helper object using the robot and
    %      % specifications for the collision objects
    %      collisionHelper = exampleHelperManipCollsions(robot);
    %
    %      % Define a configuration & check for collisions
    %      config = [0 -pi/4 pi 0.9*pi 0 -pi/2 0]';
    %      [cStatus, minDist, wpts, cBodyIdx] = checkRobotSelfCollision(collisionHelper, config);
    %
    %      % Plot results
    %      showCollision(collisionHelper, config, cBodyIdx);
    %      axis equal
    %
    %   Example using primitives and self-collision:
    %      % Load the Kinova Gen3 manipulator
    %      robot = loadrobot('kinovaGen3', 'DataFormat', 'column');
    %
    %      % Manually specify the collision cylinder primitve parameters to use
    %      collisionSpecs = {...
    %         {.05 .156 [eye(3) [0 0  .078]'; 0 0 0 1]} ... %Base
    %         {.07 .128 [eye(3) [0 0 -.064]'; 0 0 0 1]} ... %Shoulder_Link
    %         {.07 .210 [axang2rotm([1 0 0 pi/2]) [0 -.105 0]'; 0 0 0 1]} ... %HalfArm1_Link
    %         {.07 .210 [eye(3) [0 0 -.105]'; 0 0 0 1]} ... %HalfArm2_Link
    %         {.07 .208 [axang2rotm([1 0 0 pi/2]) [0 -.104 0]'; 0 0 0 1]} ... %ForeArm_Link
    %         {.07 .160 [eye(3) [0 0 -.053]'; 0 0 0 1]} ... %Wrist1_Link
    %         {.06 .160 [axang2rotm([1 0 0 pi/2]) [0 -.053 0]'; 0 0 0 1]} ... %Wrist2_Link
    %         {.09 .060 [eye(3) [0 0 -.030]'; 0 0 0 1]} ... %Bracelet_Link
    %         {.05 .050 [eye(3) [0 0  .025]'; 0 0 0 1]} ... %EndEffector_Link
    %         };
    %
    %      % Create a collision helper object using the robot and
    %      % specifications for the collision objects
    %      collisionHelper = exampleHelperManipCollsions(robot, collisionSpecs);
    %
    %      % Define a configuration & check for collisions
    %      config = [0 -pi/4 pi 0.9*pi 0 -pi/2 0]';
    %      [cStatus, minDist, wpts, cBodyIdx] = checkRobotSelfCollision(collisionHelper, config);
    %
    %      % Plot results
    %      showCollision(collisionHelper, config, cBodyIdx);
    %      axis equal
    %
    %   Example using URDF and environmental collision:
    %      % Create a collision helper object using a URDF and the
    %      % associated collision mesh path
    %      urdfPath = "iiwa14.urdf";
    %      collisionMeshPath = fullfile(matlabroot, "toolbox", "robotics", ...
    %           "robotexamples", "robotmanip", "data", "iiwa_description", ...
    %           "meshes", "iiwa14", "collision");
    %      collisionHelper = exampleHelperManipCollsions(urdfPath, collisionMeshPath, "ExhaustiveChecking", true);
    %
    %      % Create an environment of collision objects
    %      bx = collisionBox(0.5,0.5,0.25);
    %      bx.Pose = trvec2tform([-0.5 0.4 0.5]);
    %      cyl = collisionCylinder(0.1,1.5);
    %      cyl.Pose = trvec2tform([0 -0.4 0.75]);
    %      worldBodies = {bx cyl};
    %
    %      % Define a configuration and check for environment collisions
    %      config = [3*pi/4 -pi/4 -pi/2 pi/2 0 pi/2 pi]';
    %      [cStatus, minDist, wpts, cBodyIdx] = checkRobotWorldCollision(collisionHelper, config, {bx cyl});
    %
    %      % Plot results
    %      showCollision(collisionHelper, config, cBodyIdx, 'World', worldBodies);
    %      axis equal
    %
    %   See also collisionCylinder, collisionMesh, checkCollision
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties
        SourceRigidBodyTree
        
        VizRigidBodyTree
        
        RigidBodyCollisionArray
        
        %ExhaustiveChecking   - Boolean that indicates when to stop checking
        %   When the parameter is set to FALSE, the collision-checking
        %   methods exit as soon as a collision is found. When this
        %   parameter is set to TRUE, collision-checking continues between
        %   all applicable bodies, rather than exiting at the first
        %   collision.
        ExhaustiveChecking = false
    end
    
    %% Constructor methods
    
    methods
        function obj = helperManipCollsionsKINOVA(varargin)
            %exampleHelperManipulatorCollsions Constructor
            %
            %   See also collisionMesh, collisionCylinder, checkCollision
            
            narginchk(1,4);
            
            % Process optional name-value pair inputs
            if nargin > 2
                charInputs = cell(1,2);
                [charInputs{:}] = convertStringsToChars(varargin{(end-1):end});
            else
                charInputs = {};
            end
            
            names = {'ExhaustiveChecking'};
            defaults = {false};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            obj.ExhaustiveChecking = parameterValue(parser, names{1});
            
            if isa(varargin{1}, 'robotics.RigidBodyTree')
                % If the tree and primitive details are provided, construct
                % a separate primitive tree for visualization and update
                % the associated collision array
                
                % Define the tree from the input
                tree = varargin{1};
                validateattributes(tree, {'robotics.RigidBodyTree'}, {'scalar'}, 'exampleHelperManipCollsions', 'tree');
                obj.SourceRigidBodyTree = copy(tree);
                obj.VizRigidBodyTree = copy(tree);
                
                % Initialize collision array from tree
                obj.RigidBodyCollisionArray = cell(tree.NumBodies+1, 2);
                
                % Update the data format
                obj.SourceRigidBodyTree.DataFormat = 'column';
                obj.VizRigidBodyTree.DataFormat = 'column';
                
                if nargin > 1
                    % Create the collision object array using the
                    % user-provided specifications
                    collisionSpecs = varargin{2};
                    validateattributes(collisionSpecs, {'cell'}, {'2d','numel',tree.NumBodies+1}, 'exampleHelperManipCollsions', 'collisionSpecs');
                    obj.createCollisionPrimitiveArrayFromInputs(collisionSpecs);
                else
                    % Create the collision object array using the visuals
                    % in the associated rigid body tree
                    obj.createCollisionArrayFromVisuals;
                end
            else
                % If the URDF and MESHPATH are provided, import the tree
                % from the inputs and then define an array of collision
                % meshes using STLREAD on the same data
                
                urdfPath = varargin{1};
                meshPath = varargin{2};
                validateattributes(urdfPath, {'char', 'string'}, {'scalartext'}, 'exampleHelperManipCollsions', 'urdfPath');
                validateattributes(meshPath, {'char', 'string'}, {'scalartext'}, 'exampleHelperManipCollsions', 'meshPath');
                
                % Define two trees: the source tree uses the default
                % meshes, and the visualization tree uses the collision
                % meshes
                obj.SourceRigidBodyTree = importrobot(urdfPath);
                obj.SourceRigidBodyTree.DataFormat = 'column';                
                
                % Initialize collision array from tree
                obj.RigidBodyCollisionArray = cell(obj.SourceRigidBodyTree.NumBodies+1, 2);
                
                obj.VizRigidBodyTree = importrobot(urdfPath,'MeshPath', meshPath);
                obj.VizRigidBodyTree.DataFormat = 'column';
                
                % Match the meshes to the bodies based on the urdf
                obj.createCollisionMeshArrayFromURDFMeshes(urdfPath, meshPath);
                
                % Get the pose from the rigid body tree
                obj.assignCollisionPose(obj.SourceRigidBodyTree.homeConfiguration);
            end
        end
    end
    
    %% Public collision-checking methods
    
    methods        
        function [isInCollision, minimumDistance, closestBodiesWitnessPts, collisionPairIdx, closestBodiesIdx] = checkRobotSelfCollision(obj, config)
            %checkRobotSelfCollision Check collision between non-neighboring bodies
            
            % Initialize outputs
            isInCollision = false;
            minimumDistance = inf;
            collisionPairIdx = [];
            
            % Populate transformTree, which is a cell array of all body
            % transforms w.r.t. global frame
            transformTree = obj.getTransformTreeInternal(config);
            
            % Pairwise checking
            numBodies = size(obj.RigidBodyCollisionArray,1);
            for i = 1:numBodies
                for j = 1:numBodies
                    if i ~= j && i ~= j+1 && i ~= j-1 % not checking with self and neighbors
                        if ~isempty(obj.RigidBodyCollisionArray{i,1}) && ~isempty(obj.RigidBodyCollisionArray{j,1})
                            
                            % Check for collisions between all pairs of
                            % bodies. First, update the collision object
                            % pose, which is the product of the body
                            % frame's transform relative to the base, and
                            % the collision object's transform relative to
                            % the body frame.
                            obj.RigidBodyCollisionArray{i}.Pose = transformTree{i}*obj.RigidBodyCollisionArray{i,2};
                            obj.RigidBodyCollisionArray{j}.Pose = transformTree{j}*obj.RigidBodyCollisionArray{j,2};
                            [localCollisionStatus, sepDist, wPts] = checkCollision(obj.RigidBodyCollisionArray{i,1}, obj.RigidBodyCollisionArray{j,1});

                            isInCollision = isInCollision || localCollisionStatus;
                            
                            if localCollisionStatus
                                % Update dependent properties
                                minimumDistance = 0;
                                collisionPairIdx = [collisionPairIdx; [i j]]; %#ok<AGROW>
                                
                                if ~obj.ExhaustiveChecking
                                    return;
                                end
                                
                            elseif ~isInCollision
                                % If no collision has yet been detected,
                                % update the minimum distance and continue
                                % to next body pair
                                if sepDist < minimumDistance
                                    minimumDistance = sepDist;
                                    closestBodiesWitnessPts = wPts;
                                    closestBodiesIdx = [i j];
                                end
                            end
                        end
                    end
                end
            end
        end
        
        function [isInCollision, minimumDistance, closestBodiesWitnessPts, manipCollisionBodyIdx, worldCollisionBodyIdx, closestBodiesIdx, allDistances, minimumDistances, allBodiesWtnPts] = checkRobotWorldCollision(obj, config, worldCollisionObjects)
            %checkRobotWorldCollision Check collision with world
            
            % Initialize outputs
            isInCollision = false;
            manipCollisionBodyIdx = [];
            worldCollisionBodyIdx = [];
            
            % Populate transformTree, which is a cell array of all body
            % transforms w.r.t. global frame
            transformTree = obj.getTransformTreeInternal(config);
            
            % Pairwise checking
            numBodies = size(obj.RigidBodyCollisionArray,1);
            nonBaseBodies = numBodies-1;
            % End-effector link missing
            if isempty(obj.RigidBodyCollisionArray{end,1})
                nonBaseBodies = nonBaseBodies - 1;
            end
            numObstacles = numel(worldCollisionObjects);
            allDistances = zeros(nonBaseBodies*numObstacles,1);
            minimumDistances = inf * ones(numObstacles,1);
            allBodiesWtnPts = zeros(3, 2, nonBaseBodies, numObstacles );
            distIter = 1;
            for i = 1:nonBaseBodies
                
                % Iterate through all robot bodies. If no geometry is
                % assigned, skip this body
                if ~isempty(obj.RigidBodyCollisionArray{i+1})
                    obj.RigidBodyCollisionArray{i+1,1}.Pose = transformTree{i+1};
                    
                    % Iterate through all collision geometries
                    for j = 1:numel(worldCollisionObjects)
                        [localCollisionStatus, sepDist, wPts] = checkCollision(obj.RigidBodyCollisionArray{i+1,1}, worldCollisionObjects{j});
                        if localCollisionStatus
                            allDistances(distIter) = 0;
                            sepDist = 0;
                        else
                            allBodiesWtnPts(:,:,i,j) = wPts;
                            allDistances(distIter) = sepDist;
                        end
                        distIter = distIter + 1;
                        isInCollision = isInCollision || localCollisionStatus;                        
                        if localCollisionStatus
                            % Update dependent properties
                            minimumDistance = 0;
                            manipCollisionBodyIdx = [manipCollisionBodyIdx; i+1]; %#ok<AGROW>
                            worldCollisionBodyIdx = [worldCollisionBodyIdx; j]; %#ok<AGROW>
                            
                            if ~obj.ExhaustiveChecking
                                return;
                            end
                        elseif ~isInCollision
                            % If no collision has yet been detected,
                            % update the minimum distance and continue
                            % to next body pair
                            if sepDist < minimumDistances(j)
                                minimumDistances(j) = sepDist;
                                closestBodiesWitnessPts = wPts;
                                closestBodiesIdx = [i+1 j];
                            end
                        end
                    end
                end
            end
           minimumDistance=min(minimumDistances);
        end
    end
    
    %% Public helper methods
    
    methods
        function ax = showCollision(obj, config, collisionBodyIdx, varargin)
            %showCollision Plot collision results
            %   This method applies the highlightBodies method together
            %   with plot tools to quickly visualize results in two
            %   formats. The method outputs a subplot with two plots: on
            %   the left, the rigidBodyTree object specified by ROBOT is
            %   shown in configuration given by the CONFIG vector, with
            %   collision bodies highlighted. On the right, the collision
            %   geometries are visualized on their own. The method uses the
            %   array of body indices given by COLLISIONBODYIDX to
            %   determine which bodies to highlight, and highlights the
            %   bodies with the value given by HCOLOR.
            
            % Convert strings to chars
            charInputs = cell(1,nargin-3);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            
            names = {'Parent', 'VisualizationType', 'World', 'HighlightColor'};
            defaults = {[], 'CollisionTree', {}, [1 0.8 0]};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            ax = parameterValue(parser, names{1});
            vizType = parameterValue(parser, names{2});
            worldCollisionObjects = parameterValue(parser, names{3});
            hiliteColor = parameterValue(parser, names{4});
            
            % Basic input validation
            if ~isempty(ax)
                validateattributes(ax, {'Axes'}, {}, 'showCollision', 'Parent');
            end
            validateattributes(worldCollisionObjects, {'cell'}, {}, 'showCollision', 'World');
            validateattributes(hiliteColor, {'numeric'}, {'vector','numel',3}, 'showCollision', 'HighlightColor');
            vizType = validatestring(vizType, {'CollisionTree', 'CollisionObject'}, 'showCollision', 'VisualizationType');
            
            % The rigid bodies actually start at the first link, not the
            % base, so the indices have to be shifted down be 1
            validateattributes(collisionBodyIdx, {'double'}, {}, 'showCollision', 'collisionBodyIdx');
            if ~isempty(collisionBodyIdx)
                rigidBodyIdx = collisionBodyIdx-1;
            else
                rigidBodyIdx = collisionBodyIdx;
            end
            
            % Display the robot. If the CollisionTree method is selected,
            % this displays a robot that uses the collision meshes as
            % visuals, and those in collision are highlighted. If the
            % CollisionObject method is used, this displays a robot and
            % then overlays the collision objects. This does not allow the
            % collision objects to be highlighted.
            if strcmp(vizType, 'CollisionTree')
                if isempty(ax)
                    obj.showCollisionTree(config);
                    ax = gca;
                else
                    obj.showCollisionTree(config, 'Parent', ax);
                end
                hold all
                obj.highlightBodies(obj.VizRigidBodyTree, rigidBodyIdx, ax, hiliteColor);
            else
                if isempty(ax)
                    % Plot the source tree and highlight bodies in collsion
                    show(obj.SourceRigidBodyTree, config);
                    ax = gca;
                else
                    show(obj.SourceRigidBodyTree, config, 'Parent', ax);
                end
                hold all
                obj.highlightBodies(obj.SourceRigidBodyTree, rigidBodyIdx, ax, hiliteColor);
                
                % Plot the collision objects on top
                obj.showCollisionObjects(config, 'Parent', ax);
            end
            
            for i = 1:numel(worldCollisionObjects)
                show(worldCollisionObjects{i}, 'Parent', ax);
            end
        end
        
        function showCollisionTree(obj, config, varargin)
            %showCollisionTree Plot rigid body tree uses collision objects as rigid body visuals
            
            parentAxes = obj.processShowMethodInputs(nargin, varargin{:});
            
            tree = obj.VizRigidBodyTree;
            tree.show(config, 'Parent', parentAxes);
        end
        
        function showCollisionObjects(obj, config, varargin)
            %showCollisionObjects Plot collision objects using pose from rigidbodytree configuration
            
            parentAxes = obj.processShowMethodInputs(nargin, varargin{:});
            
            tree = obj.SourceRigidBodyTree;
            tree.show(config, 'Parent', parentAxes);
            hold all
            
            bodies = [{tree.Base} tree.Bodies];
            for i = 1:numel(bodies)
                % Get tree pose
                TForm = getTransform(tree, config, bodies{i}.Name, tree.Base.Name);
                
                % Get collision object information
                collisionObject = obj.RigidBodyCollisionArray{i,1};
                collisionObjectPosition = obj.RigidBodyCollisionArray{i,2};
                
                % Collision object position is a combination of the joint
                % position and the relative pose of the object to the
                % joint.
                if ~isempty(collisionObject)
                    collisionObject.Pose = TForm*collisionObjectPosition;
                    collisionObject.show('Parent',gca);
                end
            end
            
            hold off
        end
    end
    
    %% Helper Methods
    
    methods (Access = private)
        function createCollisionArrayFromVisuals(obj)
            %createCollisionArrayFromVisuals
            
            % Use the source rigid body tree to get basic info
            tree = obj.SourceRigidBodyTree;
            
            % For each of the bodies, get the body internal, which links to
            % the visual information
            allBodies = [{tree.Base} tree.Bodies];
            for i = 1:numel(allBodies)
                if i == 1
                    bodyInternal = tree.Base.BodyInternal;
                else
                    bodyInternal = tree.Bodies{i-1}.BodyInternal;
                end
                
                % Use the mesh from the visuals internal. For simplicity,
                % assume that the first visual is always the one that gets
                % used if there is more than one visual.
                visualsInternal = bodyInternal.VisualsInternal;
                if ~isempty(visualsInternal)
                    obj.RigidBodyCollisionArray{i,1} = collisionMesh(double(visualsInternal{1}.Vertices));
                    obj.RigidBodyCollisionArray{i,2} = double(visualsInternal{1}.Tform);
                end
            end
        end
        
        function createCollisionPrimitiveArrayFromInputs(obj, collisionSpecArray)
            %createCollisionPrimitiveArrayFromInputs
            
            % Use the source rigid body tree to get basic info
            tree = obj.SourceRigidBodyTree;
            
            allBodies = [{tree.Base} tree.Bodies];
            for i = 1:numel(allBodies)                
                bodyCollisionSpecs = collisionSpecArray{i};
                collisionPrimitive = collisionCylinder(bodyCollisionSpecs{1}, bodyCollisionSpecs{2});
                obj.RigidBodyCollisionArray{i,1} = collisionPrimitive;
                obj.RigidBodyCollisionArray{i,2} = bodyCollisionSpecs{3};
                
                % For visualization purposes, modify the visualization tree
                % to use cylinders as visuals
                vizTree = obj.VizRigidBodyTree;
                if i == 1
                    bodyInternal = vizTree.Base.BodyInternal;
                else
                    bodyInternal = vizTree.Bodies{i-1}.BodyInternal;
                end
                addVisualInternal(bodyInternal, 'Cylinder', [bodyCollisionSpecs{1}, bodyCollisionSpecs{2}], ...
                    bodyCollisionSpecs{3});

            end
        end
        
        function createCollisionMeshArrayFromURDFMeshes(obj, urdfPath, meshPath)
            %createCollisionMeshArrayFromURDFMeshes Create array of collision meshes
            %   This method assigns the RigidBodyCollisionArray property a
            %   value that is an array of collision meshes. These meshes
            %   are generated from STL files at the folder given by
            %   MESHPATH. This helper method finds all the meshes in the
            %   mesh folder, MESHPATH. It then reads the URDF file at the
            %   location specified by URDFPATH to associate links with mesh
            %   names. The list of available meshes is then
            %   cross-referenced with the mesh-to-link mapping to assign
            %   available meshes to the appropriate links. The resultant
            %   cell array is an array of collisionMesh objects that are
            %   ordered so they correspond directly to the order of the
            %   bodies in the RigidBodyTree. For example,
            %   obj.RigidBodyCollisionArray{3} will correspnd to the link
            %   given by obj.RigidBodyTree.BodyNames{3}.
                            
            % Get all meshes in the collision folder
            meshesStruct = dir(fullfile(meshPath, '*.stl'));
            availableMeshes = {meshesStruct.name};
            
            collisionBodies = cell(obj.SourceRigidBodyTree.NumBodies+1,2);
            treeBodies = [{obj.SourceRigidBodyTree.BaseName} obj.SourceRigidBodyTree.BodyNames];
            
            % Read the URDF and get the list of links in the rigid body
            % tree by name
            DOM = xmlread(urdfPath);
            linkElements = DOM.getElementsByTagName('link');
            
            % Iterate through the link elements and associate with a rigid
            % body
            for i = 1:linkElements.getLength
                linkName = linkElements.item(i-1).getAttribute('name');
                bodyIndex = strcmp(linkName, treeBodies);
                
                if any(bodyIndex)
                    % Get all the meshes associated with that link
                    meshes = linkElements.item(i-1).getElementsByTagName('mesh');
                    for j = 1:meshes.getLength
                        % Get the associated filename
                        fileName = meshes.item(j-1).getAttribute('filename');
                        [fileDir, name, ext] = fileparts(char(fileName));

                        % Check if the parent folder matches the collision
                        % folder. This is pretty bare-bones and are much better
                        % and more thorough ways to do this;
                        % robotics.URDFImporter uses some better internal ways
                        inputDirectory = strsplit(meshPath, {'/','\'});
                        urdfDirectory = strsplit(fileDir, {'/','\'});
                        isParentDirectorySame = strcmp(inputDirectory{end}, urdfDirectory{end});

                        % Check if the mesh matches an available mesh
                        if isParentDirectorySame
                            meshIndex = strcmp(availableMeshes, [name ext]);

                            if any(meshIndex)
                                meshData = stlread(fullfile(meshPath, availableMeshes{meshIndex}));
                                meshVertices = meshData.Points;
                                collisionBodies{bodyIndex,1} = collisionMesh(meshVertices);
                                collisionBodies{bodyIndex,2} = eye(4);
                            end
                        end
                    end
                end
            end
            
            obj.RigidBodyCollisionArray = collisionBodies;
        end
        
        function assignCollisionPose(obj, config)
            %assignCollisionPose Assign poses to collision objects
            %   This method updates the cell array of collision objects
            %   with the pose of the rigid body tree given the input
            %   specified by the CONFIG vector. This method uses the
            %   internal transform tree method, which is essentially an
            %   efficient variation on iterative calls to getTransform, and
            %   then uses that to update the Pose property of each
            %   corresponding collision object.
            
            transformTree = obj.getTransformTreeInternal(config);
            for i = 1:size(obj.RigidBodyCollisionArray,1)
                if ~isempty(obj.RigidBodyCollisionArray{i,1})
                    % The pose is the product of the body frame's transform
                    % relative to the base, and the collision object's
                    % transform relative to the body frame.
                    jointToCollisionMeshTransform = obj.RigidBodyCollisionArray{i,2};
                    obj.RigidBodyCollisionArray{i,1}.Pose = transformTree{i}*jointToCollisionMeshTransform;
                end
            end
        end
        
        function transformTree = getTransformTree(obj, config)
            %getTransformTree Compute position of each body in tree
            %   This method outputs a cell array of the transforms of each
            %   body in the associated rigidBodyTree. This method is
            %   inefficient, and an internal method below avoids redundant
            %   calls.
            
            % Initialize for the base
            transformTree = {eye(4)};
            
            % Fill in the other body transforms
            for i = 1:obj.SourceRigidBodyTree.NumBodies
                tree = obj.SourceRigidBodyTree;
                transformTree{i+1} = ... 
                    getTransform(tree, config, tree.Base.Name, tree.Bodies{i}.Name); % tree.Bodies{i} is very inefficient
            end
        end
        
        function Ttree = getTransformTreeInternal(obj, config)
            %getTransformTree Compute position of each body in tree
            %   This is an internal helper method that efficiently computes
            %   the positions of all the bodies in the rigid body tree.
            %   This is faster than computing getTransform in a loop, as
            %   that method calls each of the transforms for each body
            %   leading up to that chain for every call (so it ends up
            %   with a bunch of redundant calls).
            
            % Use an internal method and concatentate with the known
            % transform of the base, which is at the origin
            Ttree = [{eye(4)} obj.SourceRigidBodyTree.TreeInternal.forwardKinematics(config)];
        end
    end
    
    %% Static Helper methods
    
    methods (Static, Access = private)
        
        function highlightBodies(robot, bodyIndices, ax, highlightColor)
            %highlightBodies Highlight the bodies specified by the associated indices
            %   Highlight the bodies with indices given in the BODYINDICES
            %   vector on the axis AX using the color HIGHLIGHTCOLOR. The
            %   axis must already contain a visualization of the associated
            %   rigidbodytree, given by ROBOT. If no value is given for
            %   HIGHLIGHTCOLOR, an orange default value is used.
            
            if nargin < 4
                highlightColor = [1 0.5 0];
            end
            
            for i = 1:numel(bodyIndices)
                if i < 0
                    % Body is the base
                     p = findall(ax, 'type', 'patch', 'displayname', [robot.Base.Name '_mesh']);
                else
                    % Any other body
                    p = findall(ax, 'type', 'patch', 'displayname', [robot.Bodies{bodyIndices(i)}.Name '_mesh']);
                end
                
                if isempty(p)
                    continue
                else
                    p(1).FaceColor = highlightColor;
                end
            end
        end
        
        function parentAxes = processShowMethodInputs(numArgs, varargin)
            
            % Convert strings to chars
            charInputs = cell(1,numArgs-2);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            
            names = {'Parent'};
            defaults = {[]};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            parentAxes = parameterValue(parser, names{1});
                
            if isempty(parentAxes)
                figure;
                parentAxes = gca;
            end
        end
    end
end

