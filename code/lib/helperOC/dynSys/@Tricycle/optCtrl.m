function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

  %disp(class(obj.x))
  %disp(obj.x)
  
  %disp("deriv{obj.dims==1}");
  %disp(deriv{obj.dims==1});

  %ERROR 
  %disp("deriv{1}");
  %disp(deriv{1});

  %EROOR;
%% Optimal control
  p1 = deriv{obj.dims==1};
  p2 = deriv{obj.dims==2};
  p3 = deriv{obj.dims==3};

  %disp(p1)
  %x = obj.x{1};
  %y = obj.x{2};
  theta = obj.x(3);
  L = obj.L;
  delta = obj.delta;

  uOpt = cell(obj.nd, 1);


  gamma_1 = 2/3*cos(theta+delta)*p1+2/3*sin(theta+delta)*p2+(1/(3*L))*p3;
  gamma_2 = -2/3*cos(theta-delta)*p1-2/3*sin(theta-delta)*p2+(1/(3*L))*p3;
  gamma_3 = 2/3*sin(theta)*p1-2/3*cos(theta)*p2+(1/(3*L))*p3;

if strcmp(uMode, 'max')
  uOpt{1} = (gamma_1 >= 0)*obj.qRange(2) + (gamma_1 < 0)*obj.qRange(1);
  uOpt{2} = (gamma_2 >= 0)*obj.qRange(2) + (gamma_2 < 0)*obj.qRange(1);
  uOpt{3} = (gamma_3 >= 0)*obj.qRange(2) + (gamma_3 < 0)*obj.qRange(1);
  %uOpt = (deriv{obj.dims==3}>=0)*obj.wRange(2) + (deriv{obj.dims==3}<0)*(obj.wRange(1));
elseif strcmp(uMode, 'min')

  %disp("optCtrl- min called");
  uOpt{1} = (gamma_1 >= 0)*obj.qRange(1) + (gamma_1 < 0)*obj.qRange(2);
  uOpt{2} = (gamma_2 >= 0)*obj.qRange(1) + (gamma_2 < 0)*obj.qRange(2);
  uOpt{3} = (gamma_3 >= 0)*obj.qRange(1) + (gamma_3 < 0)*obj.qRange(2);
  
  %uOpt = (deriv{obj.dims==3}>=0)*(obj.wRange(1)) + (deriv{obj.dims==3}<0)*obj.wRange(2);
else
  error('Unknown uMode!')
end

end

