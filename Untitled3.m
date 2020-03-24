% s = serialport('COM5',115200)

ranges = [];

for i = 1:100
    data = sscanf(s.readline(),'| Distance top [mm]: %d |');
    ranges = [ranges, data];
end