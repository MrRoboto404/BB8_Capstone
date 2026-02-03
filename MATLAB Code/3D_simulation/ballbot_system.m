classdef ballbot_system
    % The class 'sim_struct' aids in the construction of all parameters
    % needed to make a 3D simulation. 
    % 
    % For the YZ plane, it uses the "make_ballbot" command, but otherwise
    % self-creates the XZ and XY plane matrices (until implementation is
    % made for these models)
    properties
        Axy
        Axz
        Ayz
        Bxy
        Bxz
        Byz
        Cxy
        Cxz
        Cyz
        Dxy
        Dxz
        Dyz
        vtorque
        ptorque
        ICxy
        ICxz
        ICyz
        simIn
        model_path
        workspace % model workspace
    end

    methods
        function obj = ballbot_system(icxy, icxz, icyz)
            %ballbot_system: Constructs an instance of this class

            %   calculates matrices for YZ plane
            [obj.Ayz, obj.Byz, obj.Cyz, obj.Dyz, ~] = make_ballbot();

            % loads defaults (for now, implement later) for other planes
            %[obj.Axz, obj.Bxz, obj.Cxz, obj.Dxz] = deal(zeros(4, 4));
            [obj.Axz, obj.Bxz, obj.Cxz, obj.Dxz] = make_ballbot(); % xz plane dynamics same as yz
            [obj.Axy, obj.Bxy, obj.Cxy, obj.Dxy] = deal(zeros(4, 4));

            % get input torques
            obj.vtorque = [0; 0; 0]; % individual motor torques
            obj.ptorque = reshape([0; 0; 0], [1, 3]); % planar torques in X, Y, Z

            % load initial conditions in minimal coordinates
            obj.ICxy = icxy;
            obj.ICxz = icxz;
            obj.ICyz = icyz;

            % empty stuff
            obj.simIn = []; 
            obj.model_path = [];
            obj.workspace = [];
        end

        function [obj, simIn] = create_sim(obj, model_path)
            % create sim object without loading workspace
            obj.simIn = Simulink.SimulationInput(model_path);
            simIn = obj.simIn;
        end

        function obj = load_to_sim(obj, model_path)
            % Push everything into the model workspace

            % Get model workspace (required here, can't pass obj.workspace)
            load_system(model_path);  % ensure loaded
            obj.workspace = get_param(model_path, 'ModelWorkspace');
            mw = obj.workspace;

            % XZ Plane
            mw.assignin("Axz", obj.Axz);
            mw.assignin("Bxz", obj.Bxz);
            mw.assignin("Cxz", obj.Cxz);
            mw.assignin("Dxz", obj.Dxz);
            mw.assignin("ICxz", obj.ICxz);

            % XY Plane
            mw.assignin("Axy", obj.Axy);
            mw.assignin("Bxy", obj.Bxy);
            mw.assignin("Cxy", obj.Cxy);
            mw.assignin("Dxy", obj.Dxy);
            mw.assignin("ICxy", obj.ICxy);

            % YZ Plane
            mw.assignin("Ayz", obj.Ayz);
            mw.assignin("Byz", obj.Byz);
            mw.assignin("Cyz", obj.Cyz);
            mw.assignin("Dyz", obj.Dyz);
            mw.assignin("ICyz", obj.ICyz);

            % Torques
            mw.assignin("vtorque", obj.vtorque);
            mw.assignin("ptorque", obj.ptorque);

            disp("Loaded all parameters into MODEL WORKSPACE");

        end

    end
end