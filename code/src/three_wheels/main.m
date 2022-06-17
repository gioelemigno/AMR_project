function main()
	accuracy = 'medium';
	env_type = 'dynamic';%'static'; %

	%static env
	%x0 = [-5, -8 -2];

	%dynamic_env
	%x0 = [0, -7, -3];
	%x0 = [0, 0, -3];
	x0 = [14, 14, -3];

	disturbance_strategy = 'random_normal'; %'optimal'; %'random_normal'; %'optimal'; %'random_normal'; %'optimal'; %'random_normal'; %'optimal'; %'random_normal'; %'optimal'; %'random_normal';
	three_wheels(accuracy, env_type, x0, disturbance_strategy)
end
