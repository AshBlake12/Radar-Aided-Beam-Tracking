% golden_cov.m v3 - streaming test bench for covariance_core. Feeds one
% scalar complex sample per call, in X(:) column-major order (matching
% covariance_core's documented protocol and selftest_bram.v's native
% output order), 1024 calls per 256-sample frame.

clear; M=4; K=256; SNR=20;
F=60; t=(0:F-1)'*0.1; th_true = 30*sin(2*pi*t/6);
rng(0);
worst = 0; frames_checked = 0;
for f=1:F
    X = ula(th_true(f),K,M,SNR);
    Rref = (X*X')/K;
    if f==1
        dump_hex('stim_X.hex', X);   % selftest_bram.v stimulus - ONE frame
                                       % (DEPTH=2048 words = 1024 complex = 4x256)
    end
    xr = real(X(:)); xi = imag(X(:));    % column-major: matches covariance_core protocol
    for k = 1:numel(xr)
        fs = (k==1);
        [Rtri, valid] = covariance_core(xr(k), xi(k), true, fs);
        if valid
            idx = 1; e = 0;
            for i=1:M
                for j=i:M
                    e = max(e, abs(Rtri(idx)-Rref(i,j))); idx = idx+1;
                end
            end
            worst = max(worst, e); frames_checked = frames_checked+1;
        end
    end
end
fprintf('frames checked: %d (expect %d)\n', frames_checked, F);
fprintf('covariance_core worst-case |err| vs X*X''/K: %.3e\n', worst);

function X=ula(a,K,M,snr)
    z=exp(1j*pi*sin(deg2rad(a))); A=z.^(0:M-1).';
    S=(randn(numel(a),K)+1j*randn(numel(a),K))/sqrt(2);
    n=10^(-snr/20)*(randn(M,K)+1j*randn(M,K))/sqrt(2); X=A*S+n;
end