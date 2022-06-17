function dx = dynamics(obj, ~, x, u, d)

if nargin < 5
  d = [0; 0; 0];
end

if iscell(x)
  dx = cell(length(obj.dims), 1);
  
  for i = 1:length(obj.dims)
    dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  %Dubins car
  %dx(1) = obj.speed * cos(x(3)) + d(1);
  %dx(2) = obj.speed * sin(x(3)) + d(2);
  %dx(3) = u + d(3);
  theta=x(3);
  delta=obj.delta;
  L=obj.L;
  %tricycle
  dx(1) = u(1) * 2/3 * cos(theta + delta) - u(2) * 2/3 * cos(theta-delta) + u(3) * 2/3 * sin(theta) - sin(theta) * d(1);
  dx(2) = u(1) * 2/3 * sin(theta + delta) - u(2)* 2/3 * sin(theta-delta) - u(3) * 2/3 * cos(theta) + cos(theta) * d(2);
  dx(3) = u(1) * 1/(3*L) + u(2)* 1/(3*L) + u(3) * 1/(3*L) ;
end
end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)

 % disp(class(x));
 % x_3 = x{dims==3};

 % disp(size(x_3));
switch dim
  case 1
    %disp("u1 = ");
    %disp(size(u{1}));
    %disp(u{1});
%    disp("cos(x{dims==3} + obj.delta)= ");
%    disp(size(cos(x{dims==3} + obj.delta)));

    dx = u{1} .* 2/3 .* cos(x{dims==3} + obj.delta) - u{2} .* 2/3 .* cos(x{dims==3}-obj.delta) + u{3} .* 2/3 .* sin(x{dims==3}) - sin(x{dims==3}) .* d{1};

    %disp("x=");
    %disp(size(x));
    %disp(x);

    %disp("x{dims==3}");
    %disp(size(x{dims==3}));
    %disp(x{dims==3});

    % x 3x1 (41x41x41)
    % x{dims==3} 41 x 41 x 41

    %{
    dx = u{1} * 2/3 * cos(x{dims==3} + obj.delta);
    dx = dx - u{2} * 2/3 * cos(x{dims==3}-obj.delta); 
    dx = dx + u{3} * 2/3 * sin(x{dims==3}) - sin(x{dims==3}) * d{1};
    %}
  case 2
      
    dx = u{1} .* 2/3 .* sin(x{dims==3} + obj.delta) - u{2} .* 2/3 .* sin(x{dims==3}-obj.delta) - u{3} .* 2/3 .* cos(x{dims==3}) + cos(x{dims==3}) .* d{2};
    %{
    dx = u{1} * 2/3 * sin(x{dims==3} + obj.delta); 
    dx = dx - u{2} * 2/3 * sin(x{dims==3}-obj.delta); 
    dx = dx - u{3} * 2/3 * cos(x{dims==3}) + cos(x{dims==3}) * d{2};
    %}
    %dx = obj.speed * sin(x{dims==3}) + d{2};
  case 3
    %{
    dx = u{1} * 1/(3*obj.L); 
    dx = dx + u{2} * 1/(3*obj.L); 
    dx = dx + u{3} * 1/(3*obj.L);
    %}
    dx = u{1} .* 1/(3*obj.L) + u{2} .* 1/(3*obj.L) + u{3} .* 1/(3*obj.L) ;  
    %dx = u + d{3};
  otherwise
    error('Only dimension 1-3 are defined for dynamics of Tricycle!')
end
end