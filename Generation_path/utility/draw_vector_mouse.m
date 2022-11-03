% close any open figures
close all;
clear all;
% launch a new figure and click the mouse as many time as you want within figure
disp('Click the mouse wherever in the figure; press ENTER when finished.');
mousePointCoords = ginput;
% plot the mouse point coordinates on the figure
plot(mousePointCoords(:,1), mousePointCoords(:,2),'b.','MarkerSize',8);