classdef Tricycle < DynSys
  properties
    % Angle bounds
    qRange %speed bounds on each wheel
    
    delta %costant 30 degree (wheel orientation in the Robot coordinate system)
    
    L %length (distance from center)
    
    % Disturbance
    dRange
    
    dims %dimensions that are active
  end
  
  methods
    function obj = Tricycle(x, qRange, delta, L, dRange,dims)
      % obj = DubinsCar(x, wMax, speed, dMax, dims)
      %     Dubins Car class
      %
      % Dynamics:
      %    \dot{x}_1 = v * cos(x_3) + d1
      %    \dot{x}_2 = v * sin(x_3) + d2
      %    \dot{x}_3 = u
      %         u \in [-wMax, wMax]
      %         d \in [-dMax, dMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos]
      %   thetaMin   - minimum angle
      %   thetaMax   - maximum angle
      %   v - speed
      %   dMax   - disturbance bounds
      %
      % Output:
      %   obj       - a DubinsCar2D object
      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      if nargin < 2
        qRange = [[-1;-1;-1];[1;1;1]];
      end
      
      if nargin < 3
        delta = deg2rad(30);
      end
      
      if nargin < 4
        L=1;
      end
      
      if nargin < 5
         dRange = {[0;0;0];[0; 0; 0]};
      end
      
      if nargin < 6
        dims = 1:3;
      end
      
      if numel(qRange) <2
          qRange = [[-qRange;-qRange;-qRange];[qRange;qRange;qRange]];
      end
      
      if ~iscell(dRange)
          dRange = {-dRange,dRange};
      end
      
      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      %obj.hdim = find(dims == 3);   % Heading dimensions
      obj.nx = length(dims);
      obj.nu = 3;
      obj.nd = 3;
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.qRange = qRange;
      %obj.thetaMax = thetaMax;
      obj.delta = delta;
      obj.L = L;
      obj.dRange = dRange;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef



