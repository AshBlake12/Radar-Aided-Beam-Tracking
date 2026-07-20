% compare_fixed_vs_float.m

clear; addpath(genpath('codegen'));   % generated files live under codegen/ - confirmed from your build log
M=4; P=1; K=256; SNR=20;
F=60; t=(0:F-1)'*0.1; th_true = 30*sin(2*pi*t/6);   % SAME scenario as run_golden.m
dt=0.1; Fk=[1 dt;0 1]; Hk=[1 0]; Q=[1e-2 0;0 8]; Rk=0.05;
rng(0);                                              % SAME seed -> identical X per frame

thpred_f = zeros(F,1); thpred_x = zeros(F,1);
th_f = zeros(F,1); th_x = zeros(F,1); coeff_err = zeros(F,1);
xk_f=[th_true(1);0]; Pk_f=eye(2);
xk_x=[th_true(1);0]; Pk_x=eye(2);

for f = 1:F
    X = ula(th_true(f), K, M, SNR);

    c_f = evd_core(X, P);                            % floating golden
    c_x = evd_core_wrapper_fixpt(X, P);               % generated fixed-point (requires both args - see wrapper line 9)
    coeff_err(f) = max(abs(c_f - c_x(:)));

    th_f(f) = rootmusic_angle(c_f, P);
    th_x(f) = rootmusic_angle(c_x, P);                % identical rooter, fixed-pt coeffs in

    [xk_f,Pk_f,thpred_f(f)] = kf_step(xk_f,Pk_f,th_f(f),Fk,Hk,Q,Rk);
    [xk_x,Pk_x,thpred_x(f)] = kf_step(xk_x,Pk_x,th_x(f),Fk,Hk,Q,Rk);
end

fprintf('max |coeff diff| (fixed-float)  : %.2e\n', max(coeff_err));
fprintf('RMS DOA err, float  vs truth    : %.3f deg\n', rms(th_f-th_true));
fprintf('RMS DOA err, fixed  vs truth    : %.3f deg\n', rms(th_x-th_true));
fprintf('RMS steer error, fixed vs float : %.3f deg  <- headline number\n', ...
        rms(thpred_x - thpred_f));

figure;
subplot(2,1,1); plot(t,th_true,'k',t,th_f,'b.',t,th_x,'r.'); grid on
legend('truth','float est','fixed est'); ylabel('deg'); title('DOA estimate: float vs fixed-point')
subplot(2,1,2); plot(t, thpred_x-thpred_f, 'm'); grid on
ylabel('deg'); xlabel('s'); title('steering angle error introduced by fixed-point conversion')

function X = ula(a,K,M,snr)
    z=exp(1j*pi*sin(deg2rad(a))); A=z.^(0:M-1).';
    S=(randn(numel(a),K)+1j*randn(numel(a),K))/sqrt(2);
    n=10^(-snr/20)*(randn(M,K)+1j*randn(M,K))/sqrt(2); X=A*S+n;
end

function th=rootmusic_angle(coeff,P)
    r=roots(flipud(coeff(:))); r=r(abs(r)<1);
    [~,o]=sort(abs(abs(r)-1)); r=r(o(1:P));
    th=sort(rad2deg(asin(min(max(angle(r)/pi,-1),1))));
    th=th(1);
end

function [xk,Pk,thpred] = kf_step(xk,Pk,meas,Fk,Hk,Q,Rk)
    xp=Fk*xk; Pp=Fk*Pk*Fk'+Q;
    Kg=Pp*Hk'/(Hk*Pp*Hk'+Rk);
    xk=xp+Kg*(meas-Hk*xp); Pk=(eye(2)-Kg*Hk)*Pp;
    thpred=Hk*(Fk*xk);
end