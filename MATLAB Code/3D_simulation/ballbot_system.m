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
        virt_torque
        motor_torque
        ICxy
        ICxz
        ICyz
        simIn
        model_path
        workspace % model workspace
        params % from params.m (e.g. bb8.params.m_A)
    end

    methods
        function obj = ballbot_system()
            %ballbot_system: Constructs an instance of this class

            %   calculates matrices for YZ plane
            [obj.Ayz, obj.Byz, obj.Cyz, obj.Dyz, obj.params] = make_ballbot();

            % loads defaults (for now, implement later) for other planes
            %[obj.Axz, obj.Bxz, obj.Cxz, obj.Dxz] = deal(zeros(4, 4));
            [obj.Axz, obj.Bxz, obj.Cxz, obj.Dxz] = make_ballbot(); % xz plane dynamics same as yz
            [obj.Axy, obj.Bxy, obj.Cxy, obj.Dxy] = deal(zeros(4, 4));

            % friction stuff


            % empty stuff
            obj.simIn = []; 
            obj.model_path = [];
            obj.workspace = [];

            obj.ICxy = [];
            obj.ICxz = [];
            obj.ICyz = [];
            obj.virt_torque = []; % individual motor torques
            obj.motor_torque = []; % planar torques in X, Y, Z

        end

        function [obj, simIn] = create_sim(obj, model_path)
            % create sim object without loading workspace
            obj.simIn = Simulink.SimulationInput(model_path);
            simIn = obj.simIn;
        end

        function obj = initial_conditions(obj, icxy, icxz, icyz, motortorque)
            % load initial conditions clealy in one go

            obj.ICxy = icxy;
            obj.ICxz = icxz;
            obj.ICyz = icyz;
            obj.virt_torque = []; % individual motor torques
            obj.motor_torque = motortorque; % planar torques in X, Y, Z           
        end

        function obj = load_to_sim(obj, model_path)
            disp("Beginning load...")
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
            mw.assignin("ic_phi_y", obj.ICxz(1));
            mw.assignin("ic_theta_y", obj.ICxz(2));
            mw.assignin("ic_phidot_y", obj.ICxz(3));
            mw.assignin("ic_thetadot_y", obj.ICxz(4));

            % XY Plane
            mw.assignin("Axy", obj.Axy);
            mw.assignin("Bxy", obj.Bxy);
            mw.assignin("Cxy", obj.Cxy);
            mw.assignin("Dxy", obj.Dxy);
            mw.assignin("ICxy", obj.ICxy);
            mw.assignin("ic_phi_z", obj.ICxy(1));
            mw.assignin("ic_theta_z", obj.ICxy(2));
            mw.assignin("ic_phidot_z", obj.ICxy(3));
            mw.assignin("ic_thetadot_z", obj.ICxy(4));

            % YZ Plane
            mw.assignin("Ayz", obj.Ayz);
            mw.assignin("Byz", obj.Byz);
            mw.assignin("Cyz", obj.Cyz);
            mw.assignin("Dyz", obj.Dyz);
            mw.assignin("ICyz", obj.ICyz);
            mw.assignin("ic_phi_x", obj.ICyz(1));
            mw.assignin("ic_theta_x", obj.ICyz(2));
            mw.assignin("ic_phidot_x", obj.ICyz(3));
            mw.assignin("ic_thetadot_x", obj.ICyz(4));

            % Torques
            mw.assignin("virt_torque", obj.virt_torque);
            mw.assignin("legFL_torque", obj.motor_torque(1));
            mw.assignin("legFR_torque", obj.motor_torque(2));
            mw.assignin("legB_torque", obj.motor_torque(3));


            % System parameters
            mw.assignin("r_ball", obj.params.r_ball);
            mw.assignin("m_omni", obj.params.m_omni);
            mw.assignin("m_body", obj.params.m_body);
            

            disp("Successfully loaded all parameters into model workspace.");

        end

    end
end