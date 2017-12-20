%%

for m=1:21
    if isnan(laser(m,1))
        laser(m,1)=5;
    end
end

laser
%%
[x,y]=pol2cart(laser(:,2),laser(:,1));
figure(1)
hold all
plot(x,y,'-')
plot(x,y,'*')
%%
PreS=Polyhedron('V',[x y])
PreS.plot('alpha',0.5)
%%
in=inpolygon(2,5,x,y)
%%

PreS.chebyCenter.x