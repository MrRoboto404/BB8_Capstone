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
    end

    methods
        function obj = ballbot_system(icxy, icxz, icyz)
            %ballbot_system: Constructs an instance of this class

            %   calculates matrices for YZ plane
            [obj.Ayz, obj.Byz, obj.Cyz, obj.Dyz, ~] = make_ballbot();

            % loads defaults (for now, implement later) for other planes
            [obj.Axz, obj.Bxz, obj.Cxz, obj.Dxz] = deal(0);
            [obj.Axy, obj.Bxy, obj.Cxy, obj.Dxy] = deal(0);

            % get input torques
            obj.vtorque = [0; 0; 0]; % individual motor torques
            obj.ptorque = [0; 0; 0]; % planar torques in X, Y, Z

            % load initial conditions in minimal coordinates
            obj.ICxy = icxy;
            obj.ICxz = icxz;
            obj.ICyz = icyz;
        end

        function simIn = load_to_sim(model)
            %load_to_sim: automatically loads the various parameters into
            %the given model path to store matrices, inputs, etc
            simIn = Simulink.SimulationInput(model);

            % XZ Plane
            simIn = simIn.setVariable("Axz", obj.Axz);
            simIn = simIn.setVariable("Bxz", obj.Bxz);
            simIn = simIn.setVariable("Cxz", obj.Cxz);
            simIn = simIn.setVariable("Dxz", obj.Dxz);
            simIn = simIn.setVariable("ICxz", obj.ICxz);
                       
            % XY Plane
            simIn = simIn.setVariable("Axy", obj.Axy);
            simIn = simIn.setVariable("Bxy", obj.Bxy);
            simIn = simIn.setVariable("Cxy", obj.Cxy);
            simIn = simIn.setVariable("Dxy", obj.Dxy);
            simIn = simIn.setVariable("ICxy", obj.ICxy);
            
            % YZ Plane
            simIn = simIn.setVariable("Ayz", obj.Ayz);
            simIn = simIn.setVariable("Byz", obj.Byz);
            simIn = simIn.setVariable("Cyz", obj.Cyz);
            simIn = simIn.setVariable("Dyz", obj.Dyz);
            simIn = simIn.setVariable("ICyz", obj.ICyz);

        end

    end
end