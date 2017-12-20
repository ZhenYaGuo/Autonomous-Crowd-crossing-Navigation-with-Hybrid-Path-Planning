function map = moveHumans(map, humansCurPos, humansEndPos)

% To move human in a map from cur pos to end pos
% Position inputs are in world coordinates, metres and must have same
% length

% Size refers to cell radius around human, 2 cells = 0.1 m 
radius = 6; 

numHumans = size(humansCurPos,1);

for i=1:numHumans
    % Reset space currently occupied to not occupied
    human = humansCurPos(i,:);
    setOccupancy(map,getGridSpaces(map,human(1),human(2),radius),0,'grid');
    % Set new space to occupied
    human = humansEndPos(i,:);
    setOccupancy(map,getGridSpaces(map,human(1),human(2),radius),1,'grid'); 
end

end
