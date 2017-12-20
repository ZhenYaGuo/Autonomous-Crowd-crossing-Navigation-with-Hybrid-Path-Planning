function gridSpaces = getGridSpaces(map,x,y,size)
center = world2grid(map,[x y]);
topLeft = center - [size size];
bottomRight = center + [size size];

gridSpaces = [];
for i=0:(2*size)    
    gridSpaces = [gridSpaces;(topLeft + [i 0]);(topLeft + [0 i])];
    gridSpaces = [gridSpaces;(bottomRight - [i 0]);(bottomRight - [0 i])];
end

end