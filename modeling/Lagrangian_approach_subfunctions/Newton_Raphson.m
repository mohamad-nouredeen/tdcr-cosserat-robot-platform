function [q, converged, final_residual, ...
          solver_iterations] = Newton_Raphson(Const)

    res_min = 1e-8;

    q = Const.q;
    loading_steps = Const.loading_steps;

    solver_iterations = 0;
    converged = true;
    final_residual = Inf;

    max_iterations = 100;

    for t = 1:loading_steps

        L = actuation_matrix(Const);

        tau = ...
            t*Const.tau(:)/loading_steps;

        Q_act = L*tau;

        Const.q = q;
        [Q_ext, J] = TISM(Const);

        residual = ...
            Q_ext + Const.Kee*Const.q - Q_act;

        iteration_this_step = 0;

        while norm(residual) > res_min && ...
              iteration_this_step < max_iterations

            Delta_q = ...
                (-J - Const.Kee)\residual;

            q = q + Delta_q;

            Const.q = q;
            [Q_ext, J] = TISM(Const);

            residual = ...
                Q_ext + Const.Kee*Const.q - Q_act;

            iteration_this_step = ...
                iteration_this_step + 1;

            solver_iterations = ...
                solver_iterations + 1;
        end

        final_residual = norm(residual);

        if final_residual > res_min
            converged = false;
            return;
        end

    end

end