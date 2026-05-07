function xy = grid_to_meters(rc, limits, step)
row=rc(1);
col=rc(2);

xmin=limits(1);
ymin=limits(2);

x=xmin+(col-1)*step;
y=ymin+(row-1)*step;

xy = [x, y];
end
