s = serialport('COM3',115200)

ranges = zeros(1, 171);
angles = linspace(-pi/3,pi/3,numel(ranges));

h = figure;
while ishandle(h),
    for i = 1:10
        data = sscanf(s.readline(),'%d| Distance top [mm]: %d | Count: %d');
        if length(data) == 3
            index = data(3) + 1;
            ranges(index) = data(2);
        end
    end
    scan = lidarScan(ranges,angles);

    plot(scan)
end

delete(s)


