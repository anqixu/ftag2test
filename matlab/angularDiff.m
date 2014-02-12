function [diff] = angularDiff(a, b, range)
%

d = b - a + range/2;
diff = d;
diff(d>0) = diff(d>0) - floor(diff(d>0)/range)*range - range/2;
diff(d<=0) = diff(d<=0) - (floor(diff(d<=0)/range) + 1)*range + range/2;

end
