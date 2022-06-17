function dOpt = optDstb(obj, ~, ~, deriv, dMode)

%% Input processing
if nargin < 5
  disp("wARNING: Using default dMode='max");
  dMode = 'max';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

dOpt = cell(obj.nd, 1);

theta = obj.x(3);
%% Optimal control
%{
dRange = {
      -[.25, .25, 0], % d_min
      [.25, .25, 0]   % d_max
      };

%}
disp('i');
if strcmp(dMode, 'max')  
  dOpt{1} = ((deriv{obj.dims==1}*(-sin(theta))) >= 0)*obj.dRange{2}(1) + ((deriv{obj.dims==1}*(-sin(theta))) < 0)*obj.dRange{1}(1);
  dOpt{2} = ((deriv{obj.dims==2}*(cos(theta))) >= 0)*obj.dRange{2}(2) + ((deriv{obj.dims==2}*(cos(theta))) < 0)*obj.dRange{1}(2);
  dOpt{3} = 0;
elseif strcmp(dMode, 'min')
  dOpt{1} = ((deriv{obj.dims==1}*(-sin(theta))) >= 0)*obj.dRange{1}(1) + ((deriv{obj.dims==1}*(-sin(theta))) < 0)*obj.dRange{2}(1);
  dOpt{2} = ((deriv{obj.dims==2}*(cos(theta))) >= 0)*obj.dRange{1}(2) + ((deriv{obj.dims==2}*(cos(theta))) < 0)*obj.dRange{2}(2);
  dOpt{3} = 0;
else
  error('Unknown dMode!')
end

end

