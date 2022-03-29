function robot = getRobotFromURDF(urdf_file)
% getRobotFromURDF reads a URDF file and returns a struct for the robot
%    
% Use:
%    robot = getRobotFromURDF(urdf_file)
%       - urdf_file is the path to the urdf_file
%       - robot is a struct with attributes:
%           - name
%           - n_joints
%           - n_links
%           - joints
%           - links
%       - joints is a struct-array with attributes:
%           - name
%           - type
%           - xyz
%           - rpy
%           - axis
%       - links is a struct-array with attributes:
%           - name
%           - xyz
%           - rpy

% read doc
doc_struct = readstruct(urdf_file, FileType='xml');
doc_struct = doc_struct.xacro_macro;

% append known givens
givens_map = containers.Map({'robot_name', 'parent','PI'}, ...
                            {char(doc_struct.nameAttribute), 'Origin', 'pi'});

% init robot struct
robot = struct;
robot.name     = doc_struct.nameAttribute;
robot.n_joints = length(doc_struct.joint);
robot.n_links  = length(doc_struct.link);

% add joints
robot.joints = struct;
for i = 1:robot.n_joints
    doc_joint = doc_struct.joint(i);
    
    % get joint name
    robot.joints(i).name = parseAttr(doc_joint.nameAttribute, givens_map);
    robot.joints(i).type = doc_joint.typeAttribute;

    % get joint transform
    if isa(doc_joint.origin, 'struct')
        robot.joints(i).xyz = parseAttr(doc_joint.origin.xyzAttribute, givens_map);
        robot.joints(i).rpy = parseAttr(doc_joint.origin.rpyAttribute, givens_map);
    else
        % missing origin (for fixed joints)
        robot.joints(i).xyz = nan;
        robot.joints(i).rpy = nan;
    end
    
    
    if isa(doc_joint.axis, 'struct')
        robot.joints(i).axis = parseAttr(doc_joint.axis.xyzAttribute, givens_map);
    else
        % missing axis (for fixed joints)
        robot.joints(i).axis = nan;
    end
end

% add links
robot.links = struct;
for i = 1:robot.n_links
    % get link name
    robot.links(i).name = parseAttr(doc_struct.link(i).nameAttribute, givens_map);
    
    % get link transforms
    if isa(doc_struct.link(i).inertial, 'struct')
        doc_link = doc_struct.link(i).inertial.origin;
        robot.links(i).xyz = parseAttr(doc_link.xyzAttribute, givens_map);
        robot.links(i).rpy = parseAttr(doc_link.rpyAttribute, givens_map);
    else
        robot.links(i).xyz = nan;
        robot.links(i).rpy = nan;
    end
end
end

function attr_value = parseAttr(attr_str, givens)
% parseAttr parses some strings detailed in urdf
%   Many attributes will contain the XML string '${...}' which makes life
%   difficult. This function parses those strings and replaces them with
%   their known values, specified in givens map

% convert to char for idxing
attr_str = char(attr_str);

% detect if there is a xml insert key
cnt = 0;
while contains(attr_str, '${') && cnt < 10
    [start, stop] = regexp(attr_str, '\{[^}]*', 'ONCE');
    tmp_str = attr_str(start+1:stop);
    for keys = givens.keys
        % replace known values
        key_str = keys{1};
        value_str = givens(key_str);

        tmp_str = strrep(tmp_str, key_str, value_str);
    end
    tmp_str = strrep(tmp_str, ' ','');
    attr_str = [attr_str(1:start-2) tmp_str attr_str(stop+2:end)];
    cnt = cnt+1;
end

% try to evaluate as a number
attr_value = str2num(attr_str);
% else return the string
if isempty(attr_value)
    attr_value = attr_str;
end
end