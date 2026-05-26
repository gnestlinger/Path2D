function applyPlotxyStyles(axh)
%

grid(axh, 'on');
axis(axh, 'equal');
xlabel(axh, 'x (m)');
ylabel(axh, 'y (m)');
legend(axh, 'show', 'Location','best');

end%fcn
