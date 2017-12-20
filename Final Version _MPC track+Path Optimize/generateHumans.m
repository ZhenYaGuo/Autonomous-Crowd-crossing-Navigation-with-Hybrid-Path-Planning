function [humanStartLocation, humanEndLocation] =  ...
    generateHumans(map,xLimits,yLimits)

% Returns a start location and end location which the human obstacle
% traverses

% % Old Code
% %Define first rectangle
% xLimits1 = [8.2 12];
% yLimits1 = [1.8 6.5];
% rectangle('Position', [xLimits1(1) yLimits1(1) diff(xLimits1) diff(yLimits1)]);
% [human1Start, human1End] = generateHumans(mapInflated,xLimits1,yLimits1);
% plot(human1Start,human1End,'*')
% plot(human1End(1),human1End(2),'*')

isOccupied = true;
iterations = 100;

while isOccupied && iterations > 0
    xStart = xLimits(1) + rand(1)*diff(xLimits);
    yStart = yLimits(1) + rand(1)*diff(yLimits);
    isOccupied = map.getOccupancy([xStart yStart]);
    iterations = iterations - 1;
end

isOccupied = true;
iterations = 100;

while isOccupied && iterations > 0
    xEnd = xLimits(1) + rand(1)*diff(xLimits);
    yEnd = yLimits(1) + rand(1)*diff(yLimits);
    isOccupied = map.getOccupancy([xEnd yEnd]);
    iterations = iterations - 1;
    if (~pdist([xStart,yStart;xEnd,yEnd],'euclidean') > 1.5)
        isOccupied = true;
        iterations = 100;
    end
end

humanStartLocation = [xStart, yStart];
humanEndLocation = [xEnd, yEnd];

end