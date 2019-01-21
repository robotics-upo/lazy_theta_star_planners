function mapplot(map_path, n)
%     Use: call function using the path of the plain file (it must be a
%     linear succesion of numbers, like OccupancyGrid
%     data from map_server message n is the number of columns of the
%     real map(map width in pixels)

M = vec2mat(load(map_path, '-ascii'), n);

figure;
hold on;

axis equal
title(map_path);
xm = length(M(1,:));
ym = length(M(:,1));
axis([0,xm,0,ym]);

xticks(0:xm/10:xm);
yticks(0:ym/10:ym);
grid on;

for i = 1:length(M(:,1))
    for j = 1:length(M(1,:))
        if( M(i,j) == 100 )
            plot(j-1,i-1,'k.');
        elseif( M(i,j) == -1 )
            plot(j-1,i-1,'.','color',hsv2rgb([0.5,0.5,0.5]));
        else
            plot(j-1,i-1,'.','color',hsv2rgb([(M(i,j)/100),1,1]));
        end
    end
end
    
saveas(gcf,strcat(map_path,".png"));

end