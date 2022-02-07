function [h,axh] = basicPlot(axh, obj, varargin)
%BASICPLOT  Perform basic plot operations.
%	To be used by public plot methods to avoid multiple calls to
%	plot styles like AXIS, TITLE, XLABEL, ...!
%
%	NOTE: This method supports non-scalar inputs OBJ!

    % Get current status of axes 'NextPlot' property
    if isempty(axh) || ~ishghandle(axh)
        axh = gca;
    end%if
    npState = get(axh, 'NextPlot');

    % Plot paths
    nbsObjects = builtin('numel', obj);
    h = gobjects(nbsObjects, 1);
    for i = 1:nbsObjects
        if i == 2
            set(axh, 'NextPlot', 'add');
        end%if
        
        [x,y] = getXY(obj(i));
        if strfind([varargin{:}], 'DisplayName')
            h(i) = plot(axh, x, y, varargin{:});
        else
            name = ['(',num2str(i),') ', class(obj(i))];
            h(i) = plot(axh, x, y, varargin{:}, 'DisplayName',name);
        end%if
    end%for

    if nbsObjects > 1
        legend(axh, 'show', 'Location','best');
    end%if

    % Reset axes to initial state
    set(axh, 'NextPlot',npState);

end%fcn
