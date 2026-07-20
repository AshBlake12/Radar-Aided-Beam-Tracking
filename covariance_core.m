function [Rtri, valid] = covariance_core(samp_re, samp_im, samp_valid, frame_start)

    if isempty(Rr)
        Rr = zeros(4,4); Ri = zeros(4,4);
        buf_r = zeros(4,1); buf_i = zeros(4,1);
        pos = int32(0); scount = int32(0);
    end
    if frame_start
        for i=1:4
            for j=1:4
                Rr(i,j) = 0; Ri(i,j) = 0;      % scalar resets
            end
        end
        pos(:) = int32(0); scount(:) = int32(0);
    end
    valid = false;
    Rtri = complex(zeros(10,1));
    if samp_valid
        buf_r(pos+1) = samp_re; buf_i(pos+1) = samp_im;
        if pos == 3
            for i = 1:4                          % scalar-only outer product:
                for j = 1:4                      % R(i,j) += x_i * conj(x_j)
                    Rr(i,j) = Rr(i,j) + (buf_r(i)*buf_r(j) + buf_i(i)*buf_i(j));
                    Ri(i,j) = Ri(i,j) + (buf_i(i)*buf_r(j) - buf_r(i)*buf_i(j));
                end
            end
            scount(:) = scount + 1;
            pos(:) = int32(0);
            if scount == 256
                idx = 1;
                for i=1:4
                    for j=i:4
                        Rtri(idx) = complex(Rr(i,j)*(2^-8), Ri(i,j)*(2^-8));
                        idx = idx + 1;           % *(2^-8) = /256, shift
                    end
                end
                valid = true;
                scount(:) = int32(0);
            end
        else
            pos(:) = pos + 1;
        end
    end
end