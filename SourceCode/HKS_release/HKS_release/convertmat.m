function [ret] = CONVERT(filename)

A = load(filename);
B = A.points;
C = B';
D = C(:);
[nrow, ncol] = size(D);
fid = fopen([filename, '.data'], 'w');
fwrite(fid, nrow, 'int');
fwrite(fid, D, 'double');
ret = fclose(fid);
