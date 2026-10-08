classdef HelperConvertDSPoseToSim3D < matlab.System
    % This block converts a Driving Scenario Compatible pose to 3D Simulation compatible pose. 
    % The 3D Simulation compatable pose is used by the Simulation 3D Vehicle 
    % with Ground Following block. 
    %
    % In driving scenarios compatible poses from RoadRunner Scenario, the vehicle's 
    % origin is on the ground, under the center of the rear axle. This block 
    % transforms this origin to the origin used in the 3D simulation environment, 
    % which is under the geometric center of the vehicle.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.
    
    % Copyright 2022 The MathWorks, Inc.
    properties(Nontunable)

        ActorProfiles = struct(...
                    'ActorID',1,...
                    'ClassID',1,...
                    'Length',4.7,...
                    'Width',1.8,...
                    'Height',1.4,...
                    'OriginOffset',[0 0 0],...
                    'FrontOverhang',0,...
                    'RearOverhang',0,...
                    'Wheelbase',0,...
                    'Color',[0 0 0]);

        ActorIDToConvert (1, 1) {mustBePositive, mustBeInteger} = 1
    end


    methods (Access = protected)

        function [x, y, yaw] = stepImpl(obj, actorPose)

            if isfield(actorPose,'Actors')
                actors = actorPose.Actors;
                numActors = numel(actors);
                actorIDs = ones(numActors,1);
                for kndx = 1:numActors
                    actorIDs(kndx) = actors(kndx).ActorID;
                end
                selActors = actors(actorIDs == obj.ActorIDToConvert);
                if ~isempty(selActors)
                    actor = selActors(1);
                else
                    % Error if we cannot find the specified actor
                    error("Unknown Actor ID")
                end
            else
                actor = actorPose;
            end

            % Translate position of the vehicle to mid-point. In driving
            % scenario compatible poses, vehicle origin is under the rear
            % axle and in simulation 3D, it is the midpoint of the vehicle
            % on the ground.
            
            numActorsInProfile = numel(obj.ActorProfiles);
            apIDs = ones(numActorsInProfile,1);
                
            for index = 1:numActorsInProfile
                apIDs(index) = obj.ActorProfiles(index).ActorID;
            end
            
            actorProfile = obj.ActorProfiles(apIDs == actor(1).ActorID);
            
            
            if ~isempty(actorProfile) 
                if ~isequal(actorProfile(1).OriginOffset,[0 0 0])
                    ro = actorProfile(1).Length/2 + actorProfile(1).OriginOffset(1);
                    actor(1).Position = driving.scenario.internal.translateVehiclePosition(actor(1).Position, ...
                    ro, actorProfile(1).Length, actor(1).Roll, actor(1).Pitch, actor(1).Yaw);
                end
            end
      
            x = actor(1).Position(1);
            y = actor(1).Position(2);
            yaw = actor(1).Yaw;
        end

        function flag = isInputSizeMutableImpl(~,~)
            % Return false if input size is not allowed to change while
            % system is running
            flag = false;
        end

        function num = getNumInputsImpl(~)
            % Define total number of inputs for system with optional inputs
            num = 1;
        end

        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            num = 3;
        end

        function [outx,outy,outyaw] = getOutputSizeImpl(~)
            % Return size for each output port
            outx = [1 1];
            outy = [1 1];
            outyaw = [1 1];
        end

        function [out,out2,out3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
        end

        function [out,out2,out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
        end

        function [out,out2,out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = "HelperConvert" + newline + "DSPoseToSim3D" + newline + "ActorID: " + obj.ActorIDToConvert;
        end

        function names = getInputNamesImpl(~)
            % Return input port names for System block
            names = "Actor";
        end

        function names = getOutputNamesImpl(~)
            % Return output port names for System block
            names = ["X", "Y", "Yaw"];
        end
        
    end
end
