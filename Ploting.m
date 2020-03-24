x = linspace(-2,2);
%ranges = abs((1.5).*x.^2 + 5);
%ranges(45:55) = 3.5;
angles = linspace(-pi/2,pi/2,numel(ranges));

scan = lidarScan(ranges,angles);
plot(scan)

