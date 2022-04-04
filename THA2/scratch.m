clear; clc
syms dbs dse dew dwf
syms c1 c2 c3 c4 c5 c6 c7
syms s1 s2 s3 s4 s5 s6 s7

w3 = Rot('z',c1,s1)*Rot('y',c2,s2)*[0;0;1]
q3 = [0 0 dbs].' + Rot('y',c2,s2)*[0 0 dse].'
v3 = cross(-w3, q3)


w4 = Rot('z',c1,s1)*Rot('y',c2,s2)*Rot('z',c2,s2)*[0 -1 0].'
q4 = q3
v4 = cross(-w4,q4)


%%
function R = Rot(dir, c, s)
switch (dir)
    case 'x'
        R = [1 0 0;
             0 c -s;
             0 s c];
    case 'y'
        R = [c 0 s;
             0 1 0;
            -s 0 c];
    case 'z'
        R = [c -s 0;
             s  c 0;
             0  0 1];
end
end