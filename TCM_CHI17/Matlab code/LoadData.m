

function [time, ShoulderTorq, torqs, F, R, A] = LoadData(path)
      
time = [];
ShoulderTorq = [];
torqs = [];
F = [];
R = [];
A = [];

fid = fopen(path);
if(fid == -1)
    return;
end

while ~feof(fid)
    currTime = fread(fid, 1, 'float');
    time = [time currTime];

    currTorq = [];
    currTorq = fread(fid, 1, 'float');
    ShoulderTorq = [ShoulderTorq currTorq];
    
    currF = fread(fid, 1, 'float');
    currR = fread(fid, 1, 'float');
    currA = fread(fid, 1, 'float');
    F = [F currF];
    R = [R currR];
    A = [A currA];
end
fclose(fid);