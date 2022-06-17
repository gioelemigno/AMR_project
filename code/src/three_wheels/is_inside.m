function boolean = is_inside(grid, V_x, x)
  value = eval_u(grid, V_x, x);
  
  if(value < 0)
    boolean = true;
  elseif (value == 0)
    boolean = true;
  else
    boolean = false;
  end
