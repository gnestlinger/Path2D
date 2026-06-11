function [axh,obj,dtau,opts] = parsePlotInputs(varargin)

% Init return values
axh = gobjects(0,0);
dtau = [];
opts = {};

for i = 1:nargin
    if isa(varargin{i}, 'Path2D') 
        % Handle via isa() to enter if-statement for subclasses
        obj = varargin{i};
        continue
    end
    
    switch class(varargin{i})
        case 'matlab.graphics.axis.Axes'
            axh = varargin{i};
        case {'double','single','uint8', 'uint16', 'uint32', 'uint64'}
            dtau = varargin{i};
        case 'char'
            opts = varargin(i:end);
            break
        otherwise
            error('Unsupported input!');
    end
end

end%fcn
