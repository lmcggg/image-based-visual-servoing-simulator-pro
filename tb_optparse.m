function opt = tb_optparse(in, argv)
    % Simple option parser for Toolbox functions
    if nargin == 1
        argv = {};
    end
    
    % parse the arguments
    opt = in;
    skipNext = false;
    
    for i = 1:length(argv)
        if skipNext
            skipNext = false;
            continue;
        end
        
        if ischar(argv{i}) && argv{i}(1) == '-'
            field = argv{i}(2:end);
        else
            field = argv{i};
        end
        
        % does the field exist in the input structure?
        if isfield(opt, field)
            % handle special case of a binary (logical) variable
            if islogical(in.(field))
                opt.(field) = true;
            else
                % field exists, get its value from next arg
                if i < length(argv)
                    opt.(field) = argv{i+1};
                    skipNext = true;
                end
            end
        end
    end
end 