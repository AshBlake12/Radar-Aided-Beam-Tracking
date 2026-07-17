function coeff = evd_core(X, P)                     

%PL Block

    M = size(X,1);
    R = (X*X') / size(X,2);                         % Hermitian covariance
    [d,V] = jacobi_herm(R, 10);                     % 10 sweeps: <1e-6 for M<=6
    [~,ix] = sort(d,'ascend');                      % noise = M-P smallest
    En = V(:, ix(1:M-P));
    C  = En*En';                                    % noise-subspace projector
    coeff = complex(zeros(2*M-1,1));
    idx = 1;
    for l = -(M-1):(M-1)                            % sum each diagonal of C
        s = complex(0);
        for m = 1:M
            n = m + l;
            if n>=1 && n<=M, s = s + C(m,n); end
        end
        coeff(idx) = s; idx = idx+1;
    end
end

function [d,V] = jacobi_herm(A, sweeps)

% cyclic Jacobi For a complex Hermitian matrix.

    M = size(A,1); V = complex(eye(M));
    for it = 1:sweeps
        for p = 1:M-1
            for q = p+1:M
                apq = A(p,q); mag = abs(apq);
                if mag < 1e-20, continue; end
                app = real(A(p,p)); aqq = real(A(q,q));
                phi = apq/mag;
                tau = (aqq-app)/(2*mag);
                if tau>=0, sg=1; else, sg=-1; end
                t = sg/(abs(tau)+sqrt(tau*tau+1));
                c = 1/sqrt(1+t*t); s = c*t;
                G = complex(eye(M));
                G(p,p)=c; G(q,q)=c; G(p,q)=s*phi; G(q,p)=-s*conj(phi);
                A = G'*A*G; V = V*G;
            end
        end
    end
    d = real(diag(A));
end