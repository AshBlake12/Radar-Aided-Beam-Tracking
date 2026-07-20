function dump_hex(fn, X)
    x = X(:) * 8192;                    
    q = int16(round(x));                 
    fid = fopen(fn,'w');
    for k = 1:numel(q)
        fprintf(fid,'%04X %04X\n', typecast(real(q(k)),'uint16'), ...
                                    typecast(imag(q(k)),'uint16'));
    end
    fclose(fid);
end