function J = cost_fun(X,U,Xref,A,B,Q,R)   
    J = 0;
    for i = 1 : length(U)
        u0 = U(i);
        J = J + transpose(Xref - X)*Q*(Xref - X) + u0*R*u0;    % cost function
        X = A*X + B*u0;                                        % state update
    end
end