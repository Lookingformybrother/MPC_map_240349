function rc=meters_to_grid(xy, limits, step, nRows, nCols)
x=xy(1);
y=xy(2);

xmin=limits(1);
ymin=limits(2);

col=floor((x-xmin)/step)+1;
row= floor((y-ymin)/step)+1;

col = max(1, min(nCols, col));
row= max(1, min(nRows,row));

rc = [row,col];
end
