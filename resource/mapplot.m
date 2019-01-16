function plotmap(path, n)
%     Use: call function using the path of the plain file (it must be a
%     linear succesion of numbers, like data from map_server message
%     n is the number of columns of the real map

    log = load(path, '-ascii');
    M=vec2mat(log, n);
    figure;
    hold on;
    for i=1:length(M(:,1))
        for j=1:length(M(1,:))
            if(M(i,j)==100)
                plot(j,i,'x');
            end
        end
    end
axis equal;
axis([0,length(M(1,:)),0,length(M(:,1))]);
l = length(M(1,:))/10;
b = length(M(:,1))/10;
xticks([0:l:length(M(1,:))]);
yticks([0:b:length(M(:,1))]);
grid on;
end