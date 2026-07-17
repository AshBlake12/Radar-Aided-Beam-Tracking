clear; M=4; P=1; K=256; SNR=20; d2r=@deg2rad;
F=60; t=(0:F-1)'*0.1;                       % 60 frames @ 100 ms
th_true = 30*sin(2*pi*t/6); % vehicle sweeps +/-30 deg


dt=0.1; Fk=[1 dt;0 1]; Hk=[1 0]; Q=[1e-2 0;0 8]; Rk=0.05;  % const-velocity KF

xk=[th_true(1);0]; Pk=eye(2); th_est=zeros(F,1); th_pred=zeros(F,1);
rng(0);

for f=1:F
    X = ula(th_true(f),K,M,SNR);
    coeff = evd_core(X,P);                  % <-- the PL block
    th_est(f) = rootmusic_angle(coeff,P);   % <-- PS rooting (MATLAB ref)
    xp=Fk*xk; Pp=Fk*Pk*Fk'+Q;               % KF predict
    Kg=Pp*Hk'/(Hk*Pp*Hk'+Rk);               % KF update with measured angle
    xk=xp+Kg*(th_est(f)-Hk*xp); Pk=(eye(2)-Kg*Hk)*Pp;
    th_pred(f)=Hk*(Fk*xk);                  % steer to NEXT frame position
   
    if f==1
        assert(max(abs(sort(myeig(X))-sort(realeig(X))))<1e-6,'EVD mismatch');
        dump_hex('stim_X.hex',X);
    end

   C(f,:) = reshape([real(coeff).' ; imag(coeff).'], 1, []);  %#ok per-frame coeff row
end

writematrix(C,'frames.csv');
writematrix([t th_true th_est th_pred],'golden.csv');
w = exp(1j*pi*sin(d2r(th_pred)));           % AD9361 TX2 baseband weight
fprintf('RMS DOA err %.3f deg | mean |steer error| %.3f deg\n',...
        rms(th_est-th_true), mean(abs(th_pred(1:end-1)-th_true(2:end))));


plot(t,th_true,'k',t,th_est,'.',t,th_pred,'r'); legend('true','est','predicted');
xlabel('s'); ylabel('azimuth (deg)'); grid on
 

function X=ula(a,K,M,snr)
    z=exp(1j*pi*sin(deg2rad(a))); A=z.^(0:M-1).';
    S=(randn(numel(a),K)+1j*randn(numel(a),K))/sqrt(2);
    n=10^(-snr/20)*(randn(M,K)+1j*randn(M,K))/sqrt(2); X=A*S+n;
end

function th=rootmusic_angle(coeff,P)
    r=roots(flipud(coeff)); r=r(abs(r)<1);
    [~,o]=sort(abs(abs(r)-1)); r=r(o(1:P));
    th=sort(rad2deg(asin(min(max(angle(r)/pi,-1),1))));
    th=th(1);                               % primary target
end
function e=myeig(X), c=evd_core_eigsonly(X); e=c; end

function e=realeig(X), R=(X*X')/size(X,2); e=eig(R); end

function e=evd_core_eigsonly(X)
    R=(X*X')/size(X,2); [d,~]=local_jac(R); e=d;
end

function [d,V]=local_jac(A)                  % same rotation as evd_core
    M=size(A,1); V=complex(eye(M));
    for it=1:10, for p=1:M-1, for q=p+1:M
        apq=A(p,q); m=abs(apq); if m<1e-20, continue; end
        app=real(A(p,p)); aqq=real(A(q,q)); phi=apq/m; tau=(aqq-app)/(2*m);
        sg=1; if tau<0, sg=-1; end; tt=sg/(abs(tau)+sqrt(tau*tau+1));
        c=1/sqrt(1+tt*tt); s=c*tt; G=complex(eye(M));
        G(p,p)=c; G(q,q)=c; G(p,q)=s*phi; G(q,p)=-s*conj(phi); A=G'*A*G; V=V*G;
    end, end, end
    d=real(diag(A));
end

function dump_hex(fn,X)                      % Q1.15 interleaved re,im
    x=X(:)/max(abs(X(:)))*0.98; q=int16(round(x*32768));
    fid=fopen(fn,'w');
    for k=1:numel(q)
        fprintf(fid,'%04X %04X\n',typecast(real(q(k)),'uint16'),...
                                  typecast(imag(q(k)),'uint16'));
    end
    fclose(fid);
end

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