classdef SerialChain
    %SERIALCHAIN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        filename
    end
    
    methods
        function obj = SerialChain(urdf_file)
            %SERIALCHAIN Construct an instance of this class
            %   Detailed explanation goes here
            obj.filename = urdf_file;
        end
        
        function getSerialChain(obj)
            

        end
    end
end

