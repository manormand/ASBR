function reduced_mat = reduceMatrix(M)
% reduceMatrix Use elementary row operations to reduce the given matrix, M

[m,n] = size(M);
M_rank = rank(M);

% Elininate lower triangle
for i = 1:n
    % If rank deficient, rows are zero
    if i>M_rank
        M(i:m,:) = 0;
        break;
    end

    % Make sure main diagonal is nonzero
    k = i+1;
    while M(i,i)==0 && k<=m
        M = swapRows(M,i,k);
        k = k+1;
    end

    % normalize row with pivot
    M(i,:) = M(i,:)/M(i,i);

    % make column elements zero using pivot
    for j = [1:i-1 i+1:m]
        M(j,:) = M(j,:) - (M(j,i)/M(i,i))*M(i,:);
    end
end

reduced_mat = M;
end

function M = swapRows(M,row1,row2)
% swap rows
tmp = M(row1,:);
M(row1,:) = M(row2,:);
M(row2,:) = tmp;
end