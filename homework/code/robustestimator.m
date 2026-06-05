clear;

% sets simulation parameters
n = 5;
h_nom = ones(n,1); 
h_nom = h_nom/norm(h_nom);
A = diag([ 1.4 ; 0.3 ; 0.8 ; 0.1 ; 0.1 ]);
P = A*A;

% computes g_naive
g_naive = h_nom/norm(h_nom)^2;

% computes optimal g
% insert your code here
cvx_begin quiet
    variable g_robust(n)
    variable alph
    variable bet

    minimize( alph + bet )

    subject to
        g_robust' * h_nom - 1 <= alph;
       -g_robust' * h_nom + 1 <= alph;
        norm(A * g_robust, 2) <= bet;
cvx_end

% at the end of your code, the variable g_robust should contain 
% your optimal g

% runs simulations to compare the relative errors of both estimators
% (don't change anything from this line below)
MC = 100;

relative_error_naive = [];
relative_error_robust = [];

for m = 1:MC
    
    theta = randn(1);
    
    u = randn(n,1);
    u = u/norm(u);
    
    y = (h_nom + A * u) * theta;
    
    theta_naive = g_naive' * y;
    theta_robust = g_robust' * y;
    
    relative_error_naive = [ relative_error_naive , abs(theta_naive-theta)/abs(theta) ];
    relative_error_robust = [ relative_error_robust , abs(theta_robust-theta)/abs(theta) ];
end

figure(1); clf;
plot(relative_error_naive,'r-'); hold on;
plot(relative_error_robust,'b-'); 
title('Methods: naive (red) and robust (blue)');
ylabel('Relative error');
xlabel('Channel realization');
