function pivotCalibration(Fk)
n = 1
A = [];
b = [];
for k = 1:n
    R = Fk(1:3,1:3,k);
    p = Fk(1:3,4,k);
    
    A = [A; R -eye(3)];
    b = [b; -p];
end

A\b