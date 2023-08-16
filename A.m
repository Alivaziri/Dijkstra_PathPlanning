clc;
clear;
%inputing data from txt
input = fopen('input.txt','r');
S_point = fscanf(input,'%f, %f\n',[2 1]);
E_point = fscanf(input,'%f, %f\n',[2 1]);
Num_obs = fscanf(input,'%d',1);
obstacles = fscanf(input,'%f ,%f ,%f ,%f\n',[2 Num_obs*2]);
fclose(input);
%achieving points and obstacles lines
Num_points = 2 + 2*Num_obs;
point(:,1) = S_point;
point(:,Num_points) = E_point;
j = 1;
obs_poly = zeros(Num_obs,2);
for i = 1:2:Num_obs*2
    point(:,i+1) = obstacles(:,i);
    point(:,i+2) = obstacles(:,i+1);
    obs_poly(j,:) = polyfit([obstacles(1,i), obstacles(1,i+1)],[obstacles(2,i), obstacles(2,i+1)],1);
    j = j + 1;
end
%dstnce between 2 points if its possible
dstnce = zeros(Num_points);
for i = 1:Num_points
    for j = 1:Num_points
        if i<=j
            collision = 0;
            if point(:,i) == point(:,j)
                dstnce(i,j) = NaN;
            else
                points_poly = polyfit([point(1,i),point(1,j)],[point(2,i),point(2,j)],1);
                for k = 1:Num_obs
                    if points_poly(1) == obs_poly(k,1)
                        if points_poly(2) == obs_poly(k,2)
                            collision = 0;
                            break
                        else
                        collision = 0;
                        continue;
                        end
                    end
                    x = (points_poly(2)-obs_poly(k,2))/(obs_poly(k,1)-points_poly(1));
                    y = obs_poly(k,1)*x+obs_poly(k,2);
                    if obstacles(1,2*k) == obstacles(1,2*k-1)
                        x = obstacles(1,2*k);
                        y= points_poly(1)*x+points_poly(2);
                    end
                    if point(:,i) == obstacles(:,2*k-1)
                        continue;
                    elseif point(:,i) == obstacles(:,2*k)
                        continue;
                    end
                    if abs(norm([x;y]-obstacles(:,2*k-1))+norm([x;y]-obstacles(:,2*k))-norm(obstacles(:,2*k-1)-obstacles(:,2*k))) <=0.0001
                        if norm([x;y]-point(:,j))> norm(point(:,i)-point(:,j))
                            collision = 0;
                            continue;
                        end
                        if point(:,j) == obstacles(:,2*k-1)
                            collision = 0;
                            continue;
                        elseif point(:,j) == obstacles(:,2*k)
                            collision = 0;
                            continue;
                        else
                            collision = 1;
                            break;
                        end
                    end
                end
                if collision ~= 0
                    dstnce(i,j) = NaN;
                else
                    dstnce(i,j) = norm(point(:,i)-point(:,j));
                end
            end
        end
    end
end
for i = 1:Num_points
    for j = 1: Num_points
        if i<=j
            dstnce(j,i) = dstnce(i,j);
        end
    end
end
%putting points infos such as dstnce between the point and the end_point
point_info = zeros(2,Num_points);
for i = 1:Num_points
    point_info(1,i) = i;
    point_info(2,i) = inf;
    if i == Num_points
        point_info(2,i) = 0;
    end
end
%calculating the best point informations in order
point_order = zeros(2,Num_points);
for i = 1:Num_points
    active = min(point_info(2,:));
    for p_active = 1:Num_points
        if point_info(2,p_active) == active
            break;
        end
    end
    for j = 1:Num_points
        if ~isnan(dstnce(p_active,j)) && ~isnan(point_info(2,j)) && point_info(2,j)>= dstnce(p_active,j)+point_info(2,p_active)
            point_info(2,j) = dstnce(p_active,j)+point_info(2,p_active);
        end
    end
    point_order(:,p_active) = point_info(:,p_active);
    point_info(2,p_active) = NaN;
end
%calculating the best and fastest path to move from s to e by point_order
%into moving_order
path_point = point_order(1,1);
moving_order(1) = path_point;
j = 2;
while(1)
    for i = 1:Num_points
        if point_order(2,path_point) == point_order(2,i)+dstnce(path_point,i) && i ~= path_point
            path_point = i;
            moving_order(j) = path_point;
            j = j+1;
        end
    end
    if path_point == point_order(1,Num_points)
        break
    end    
end
%showing points in move
for i = 1:length(moving_order)
    x = sprintf('point %d:\n',i);
    disp(x);
    disp(point(:,moving_order(i)));
end
%plotting the way
for i = 1:length(moving_order)-1
    plot([point(1,moving_order(i)),point(1,moving_order(i+1))],[point(2,moving_order(i)),point(2,moving_order(i+1))]);
    if i == 1
        hold on;
    end
end
for i = 1:2:Num_obs*2
    plot([obstacles(1,i),obstacles(1,i+1)],[obstacles(2,i),obstacles(2,i+1)])
end