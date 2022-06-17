

function [c_lower, c_upper] = to_C_space(w_lower, w_upper, radius, type_set)
  wlx = w_lower(1);
  wly = w_lower(2);
  
  wux = w_upper(1);
  wuy = w_upper(2);


  if strcmp(type_set, 'reach_set')
    clx = wlx + radius;
    cly = wly + radius;

    cux = wux - radius;
    cuy = wuy - radius;
  elseif strcmp(type_set, 'avoid_set')
    clx = wlx - radius;
    cly = wly - radius;

    cux = wux + radius;
    cuy = wuy + radius;
  else
    disp("unknown type_set");
    c_lower = w_lower;
    c_upper = w_upper;
  end

  c_lower = [clx, cly];
  c_upper = [cux, cuy];
end