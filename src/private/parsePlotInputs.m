function [ax,obj,dtau,opts] = parsePlotInputs(varargin)

indAx = 0;
indDtau = 0;
indOpts = 0;
for i = 1:nargin
    if isa(varargin{i}, 'Path2D') 
        % Handle via isa() to enter if-statement for subclasses
        indObj = i;
        continue
    end

    switch class(varargin{i})
        case 'matlab.graphics.axis.Axes'
            indAx = i;
        case {'double','single','uint8', 'uint16', 'uint32', 'uint64'}
            indDtau = i;
        case 'char'
            indOpts = i;
            break
        otherwise
            error('Unsupported input!');
    end
end

obj = varargin{indObj};

ax = [];
dtau = [];
opts = {};
if indAx > 0
    ax = varargin{indAx};
end
if indDtau > 0
    dtau = varargin{indDtau};
end
if indOpts > 0
    opts = varargin(indOpts:end);
end

end%fcn