function plot_three_wheel(x_r, y_r, theta, L, color, color_alpha)
    if nargin < 6
        color_alpha = 0.5;
    end
    R_x = x_r;
    R_y = y_r;
    %L = 1;
    u = L / 5.25;

    theta = rad2deg(theta);

    x_a = R_x - L;
    y_a = R_y;

    x_b = x_a;
    y_b = y_a + 0.45*u;

    x_c = x_a + 7*u;
    y_c = y_a + 5*u;

    x_d = x_c + 1.75*u;
    y_d = y_c - 0.75*u;

    x_e = x_c;
    y_e = y_c - 3.5*u;

    x_f = x_a + 7*u;
    y_f = y_a;


    x_b1 = x_b;
    y_b1 = y_b - 0.45*2*u;

    x_c1 = x_c;
    y_c1 = y_c - 5*2*u;

    x_d1 = x_d;
    y_d1 = y_d - 4.25*2*u;

    x_e1 = x_e;
    y_e1 = y_e - 1.5*2*u;


    %f = figure;

    hold on;

    x = [x_b x_c x_d x_e x_e1 x_d1 x_c1 x_b1];
    y = [y_b y_c y_d y_e y_e1 y_d1 y_c1 y_b1];

    %theta = 90;
    ps = polyshape(x, y);
    ps = rotate(ps, theta, [R_x R_y]);
    pl = plot(ps);
    pl.FaceColor = color;
    pl.FaceAlpha = color_alpha;

    x_q1 = x_b;
    y_q1 = y_b;

    x_q2 = R_x;
    y_q2 = R_y + 0.45*u;

    x_q3 = R_x;
    y_q3 = R_y - 0.45*u;

    x_q4 = x_b1;
    y_q4 = y_b1;

    x = [x_q1 x_q2 x_q3 x_q4];
    y = [y_q1 y_q2 y_q3 y_q4];
    ps = polyshape(x, y);
    ps = rotate(ps, theta, [R_x R_y]);
    pl = plot(ps);
    pl.FaceColor = 'black';
    pl.FaceAlpha = color_alpha;
    hold off;


%    grid on;
%    pbaspect([1 1 1]);