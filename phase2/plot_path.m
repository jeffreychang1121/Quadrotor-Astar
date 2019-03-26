function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
plot3(path(:,1)',path(:,2)',path(:,3)');
hold on
grid on

x_low=map.blocks(:,1);
y_low=map.blocks(:,2);
z_low=map.blocks(:,3);
x_high=map.blocks(:,4);
y_high=map.blocks(:,5);
z_high=map.blocks(:,6);
color=map.blocks(:,7:9);

for i = 1:size(x_low,1) 
x=[x_low(i) x_high(i) x_high(i) x_low(i) x_low(i) x_low(i) x_low(i) x_low(i) x_low(i) x_low(i) x_low(i) x_high(i) x_high(i) x_high(i) x_high(i) x_high(i) x_high(i) x_low(i)];
y=[y_low(i) y_low(i) y_low(i) y_low(i) y_low(i) y_low(i) y_high(i) y_high(i) y_low(i) y_high(i) y_high(i) y_high(i) y_high(i) y_low(i) y_low(i) y_high(i) y_high(i) y_high(i)];
z=[z_low(i) z_low(i) z_high(i) z_high(i) z_low(i) z_high(i) z_high(i) z_low(i) z_low(i) z_low(i) z_high(i) z_high(i) z_low(i) z_low(i) z_high(i) z_high(i) z_low(i) z_low(i)];
plot3(x,y,z,'color',color(i,:)/255);
hold on
fill3([x_low(i) x_high(i) x_high(i) x_low(i)],[y_low(i) y_low(i) y_low(i) y_low(i)],[z_low(i) z_low(i) z_high(i) z_high(i)],color(i,:)/255);
fill3([x_low(i) x_low(i) x_low(i) x_low(i) ],[y_low(i) y_high(i) y_high(i) y_low(i)],[z_high(i) z_high(i) z_low(i) z_low(i)],color(i,:)/255);
fill3([x_low(i) x_low(i) x_high(i) x_high(i)],[y_high(i) y_high(i) y_high(i) y_high(i)],[z_low(i) z_high(i) z_high(i) z_low(i)],color(i,:)/255);
fill3([x_high(i) x_high(i) x_high(i) x_high(i) ],[y_low(i) y_low(i) y_high(i) y_high(i)],[z_low(i) z_high(i) z_high(i) z_low(i)],color(i,:)/255);
fill3([x_low(i) x_low(i) x_high(i) x_high(i)],[y_low(i) y_high(i) y_high(i) y_low(i)],[z_high(i) z_high(i) z_high(i) z_high(i)],color(i,:)/255);
fill3([x_low(i) x_low(i) x_high(i) x_high(i) ],[y_low(i) y_high(i) y_high(i) y_low(i)],[z_low(i) z_low(i) z_low(i) z_low(i)],color(i,:)/255);
end
end