function myNLMPCSetup()
    
    % 1) Dimensions: 2 states, 2 measured outputs, 1 input
    nx = 2;  % states: x(1)=v, x(2)=SoC
    ny = 2;  % outputs: v, SoC
    nu = 1;  % single input: 'regen'
    
    % 2) Create nlmpc object
    nlobj = nlmpc(nx, ny, nu);

    % 3) Controller sample time & horizons
    nlobj.Ts                 = 0.1;   % sample time (s)
    nlobj.PredictionHorizon  = 10;    % # of future steps to predict
    nlobj.ControlHorizon     = 2;     % # of manipulated moves in that horizon

    % 4) State and output functions (below)
    nlobj.Model.StateFcn  = @myStateFcn;
    nlobj.Model.OutputFcn = @myOutputFcn;


    % 5) Constraints for 2 outputs (v, SoC)
    %   Example for a Formula Student EV:
    %   - Speed >= 0 (no reverse in this example)
    %   - SoC between 0.2 and 1.0
    nlobj.OV(1).Min = 0;      % v >= 0
    nlobj.OV(2).Min = 0.2;    % SoC >= 0.2
    nlobj.OV(2).Max = 1.0;    % SoC <= 1.0

    % 6) Constraints on the single MV (regen)
    nlobj.MV.Min = -100;  % some negative limit (strong braking)
    nlobj.MV.Max =  100;  % some positive limit (motoring or mild regen)

    % 7) Weights (tune how strongly we track v & SoC, and penalize MV changes)
    nlobj.Weights.OutputVariables          = [1 1];   % track v & SoC equally
    nlobj.Weights.ManipulatedVariables     = 0;       % cost on absolute regen
    nlobj.Weights.ManipulatedVariablesRate = 0.1;     % cost on rate of change

    % 8) Export to base workspace
    assignin('base','nlobj',nlobj);
    disp('Created "nlobj" in base workspace for 2-output (v, SoC) racing EV.');
end

%--------------------------
% Local Functions
%--------------------------
function xdot = myStateFcn(x,u)
    % myStateFcn: Returns [dv; dSoC] for  EV.
    %
    %  x(1) = v (m/s)
    %  x(2) = SoC (0..1)
    %  u(1) = regen (Nm or some torque command), negative -> braking, positive -> drive
   

    %parameters:
    m       = 300;     % car mass (kg),
    r       = 0.2;     % wheel radius (m)
    rho     = 1.225;   % air density (kg/m^3)
    Cd      = 0.5;     % drag coefficient 
    A       = 1.2;     % frontal area (m^2)
    g       = 9.81;    
    Crr     = 0.015;   % rolling resistance
    eta_chg = 0.85;    % regen efficiency
    Enom    = 5e5;     % nominal battery energy scale (example)

    v     = x(1);
    SoC   = x(2);
    regen = u(1);

    % Speed derivative:
    dv   = -(1/m)*((regen/r) + 0.5*rho*Cd*A*v^2 + m*g*Crr);
    % SoC derivative (regeneration effect):
    dSoC = eta_chg*(regen/Enom)*(v/r);

    xdot = [dv; dSoC];
end

function y = myOutputFcn(x,u)
    % Outputs: [v; SoC]
    y = x;  
end
