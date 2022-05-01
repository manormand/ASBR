function output = getDataset(log_id)
% loadDataset
clc
disp(['======= Loading dataset for ''' log_id ''' ======='])
output.id = log_id;

% generate prefix based on id
prefix = 'pa1-debug-';

unkown = any(log_id=='hjk');
if unkown
    prefix = 'pa1-unkown-';
end

% create path
filename = [prefix log_id '-'];
log_files_prefix = fullfile('data_given', filename);

% read file according to type
output.calbody     = readCalbody(log_files_prefix);
output.calreadings = readCalreadings(log_files_prefix);
output.empivot     = readEMPivot(log_files_prefix);
output.optpivot    = readOptPivot(log_files_prefix);
if ~unkown
    output.output1 = readOutput1(log_files_prefix);
end

disp('=======================================')

end % function end


%% loading functions for fileptypes
function output = readCalbody(files_prefix)
% Parse file to a struct, output
    fprintf('Loading calbody.......')

    output = struct;
    
    % get data from file
    filename = [files_prefix 'calbody.txt'];
    file_ID = fopen(filename, 'r');
    
    header = split(fgetl(file_ID), ', ');
    
    fspec = '%f, %f, %f';
    data = fscanf(file_ID, fspec, [3 inf])';
    
    fclose(file_ID);
    
    % extract number of points
    ND = str2double(header{1});
    NA = str2double(header{2});
    NC = str2double(header{3});
    
    idx1 = ND;
    idx2 = idx1 + NA;
    idx3 = idx2 + NC;
    
    % extract points as a set
    output.d = data( 1:idx1, : );
    output.a = data( (idx1+1):idx2, : );
    output.c = data( (idx2+1):idx3, : );
    
    fprintf('DONE!\n')
end


function output = readCalreadings(files_prefix)
% Parse file to a struct, output
    fprintf('Loading calreadings...')

    output = struct;

    % get data from file
    filename = [files_prefix 'calreadings.txt'];
    file_ID = fopen(filename, 'r');
    
    header = split(fgetl(file_ID), ', ');
    
    fspec = '%f, %f, %f';
    data = fscanf(file_ID, fspec, [3 inf])';
    
    fclose(file_ID);
    
    % extract number of points
    n_d = str2double(header{1});
    n_a = str2double(header{2});
    n_c = str2double(header{3});
    n_frames = str2double(header{4});
    
    n_points = n_d + n_a + n_c;
    
    % setup loop vars
    idx1 = n_d;
    idx2 = idx1 + n_a;
    idx3 = idx2 + n_c;
    
    D = zeros(n_d, 3, n_frames);
    A = zeros(n_a, 3, n_frames);
    C = zeros(n_c, 3, n_frames);
    
    % get data for every frame
    for k = 1:n_frames
        start = (k-1)*n_points + 1;
        stop = k*n_points;
        data_k = data(start:stop, :);
        
        D(:,:,k) = data_k( 1:idx1, : );
        A(:,:,k) = data_k( (idx1+1):idx2, : );
        C(:,:,k) = data_k( (idx2+1):idx3, : );
    end

    % to struct
    output.D = D;
    output.A = A;
    output.C = C;

    fprintf('DONE!\n')
end

function output = readEMPivot(files_prefix)
% Parse file to a struct, output
    fprintf('Loading empivot.......')
    output = struct;

    
    % get data from file
    filename = [files_prefix 'empivot.txt'];
    file_ID = fopen(filename, 'r');
    
    header = split(fgetl(file_ID), ', ');
    
    fspec = '%f, %f, %f';
    data = fscanf(file_ID, fspec, [3 inf])';
    
    fclose(file_ID);
    
    % extract number of points
    n_g = str2double(header{1});
    n_frames = str2double(header{2});
    
    % setup loop vars
    G = zeros(n_g, 3, n_frames);
    
    % get data for every frame
    for k = 1:n_frames
        start = (k-1)*n_g + 1;
        stop = k*n_g;
        
        G(:,:,k) = data(start:stop,:);
    end
    
    % to struct
    output.G = G;

    fprintf('DONE!\n')
end

function output = readOptPivot(files_prefix)
% Parse file to a struct, output
    fprintf('Loading optpivot......')

    output = struct;
    
    % get data from file
    filename = [files_prefix 'optpivot.txt'];
    file_ID = fopen(filename, 'r');
    
    header = split(fgetl(file_ID), ', ');
    
    fspec = '%f, %f, %f';
    data = fscanf(file_ID, fspec, [3 inf])';
    
    fclose(file_ID);
    
    % extract number of points
    n_d = str2double(header{1});
    n_h = str2double(header{2});
    n_frames = str2double(header{3});
    
    n_points = n_d + n_h;
    
    % setup loop vars
    idx1 = n_d;
    idx2 = idx1 + n_h;
    
    D = zeros(n_d, 3, n_frames);
    H = zeros(n_h, 3, n_frames);
    
    % get data for every frame
    for k = 1:n_frames
        start = (k-1)*n_points + 1;
        stop = k*n_points;
        data_k = data(start:stop, :);
        
        D(:,:,k) = data_k( 1:idx1, : );
        H(:,:,k) = data_k( (idx1+1):idx2, : );
    end

    % to struct
    output.D = D;
    output.H = H;

    fprintf('DONE!\n')
end

function output = readOutput1(files_prefix)
% Parse file to a struct, output
    fprintf('Loading output1.......')

    output = struct;
    
    % get data from file
    filename = [files_prefix 'output1.txt'];
    file_ID = fopen(filename, 'r');
    
    header = split(fgetl(file_ID), ', ');
    
    fspec = '%f, %f, %f';
    data = fscanf(file_ID, fspec, [3 inf])';
    
    fclose(file_ID);
    
    % extract number of points
    n_g = str2double(header{1});
    n_frames = str2double(header{2});
    
    % setup loop vars
    C = zeros(n_g, 3, n_frames);
    
    % get data for every frame
    for k = 1:n_frames
        start = (k-1)*n_g + 1;
        stop = k*n_g;
        
        C(:,:,k) = data(start:stop,:);
    end
    
    % to struct
    output.C = C;

    fprintf('DONE!\n')
end