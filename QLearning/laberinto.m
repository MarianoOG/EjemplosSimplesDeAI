function [ ] = laberinto()
    % Define maze (0 - empty space, 1 - wall):
    maze = [0,0,0,1,0,0,0,0,0,0,0,0;
            0,0,0,1,0,0,1,1,1,1,1,1;
            1,0,0,1,0,0,0,0,0,0,0,0;
            0,0,0,0,0,0,0,0,0,0,0,0;
            0,1,1,0,0,1,0,0,1,1,0,1;
            0,0,1,0,0,1,0,0,1,0,0,0;
            0,0,1,0,0,0,0,0,1,0,0,0;
            0,0,1,0,0,0,0,0,1,0,0,0;
            0,0,1,1,1,1,1,0,1,0,1,1;
            1,0,0,0,0,0,0,0,1,0,0,0;
            0,0,0,1,1,1,1,0,1,0,0,0;
            0,0,0,1,0,0,0,0,0,0,0,0];
	start = [12,7];
    goal = [1,6];
    
    figure(1), subplot(1,2,1)
        imshow(maze,[],'InitialMagnification',3000);
        title('Original Maze')
        
    % Change to linear indices:
    start = sub2ind(size(maze),start(1),start(2));
    goal = sub2ind(size(maze),goal(1),goal(2));
    
    % Solve the maze:
    R = mazeRewards(maze,start,goal);
    gamma = 0.9;
    [Q,n] = qlearning(R,gamma);
    pi = policy(Q,start,goal);
    
    % Update maze:
    maze(pi) = 0.5;
    maze(start) = 0.25;
    maze(goal) = 0.75;
    
    % Graphics:
    figure(1), subplot(1,2,2)
        imshow(maze,[],'InitialMagnification',3000);
        title(['Maze solved in ',num2str(n),' episodes'])
end

function [R] = mazeRewards(maze,start,goal)
    [a,b] = size(maze);
    R = -1*ones(a*b);
    for i = 1:a
        for j = 1:b
            p = sub2ind([a,b],i,j);
            if(j~=1 && maze(i,j-1)~=1)
                R(p,sub2ind([a,b],i,j-1)) = 0;
            end
            if(i~=1 && maze(i-1,j)~=1)
                R(p,sub2ind([a,b],i-1,j)) = 0;
            end
            if(j~=b && maze(i,j+1)~=1)
                R(p,sub2ind([a,b],i,j+1)) = 0;
            end
            if(i~=a && maze(i+1,j)~=1)
                R(p,sub2ind([a,b],i+1,j)) = 0;
            end
        end
    end
    for i = 1:a*b
        if(R(i,goal)==0), R(i,goal)=100; end
        if(R(i,start)==0), R(i,start)=-100; end
    end
end

function [Q,n] = qlearning(R,gamma)
    n = 0;
    [a,b] = size(R);
    Q = zeros(a,b);
    Qant = Q;
    while(true)
        for i = 1:a
            for j = 1:b
                if(R(i,j)~=-1)
                    Q(i,j) = R(i,j) + gamma*max(Q(j,:));
                end
            end
        end
        n = n + 1;
%         disp('Q:');
%         disp(Q);
        if(sum(sum(abs(Qant-Q)<1e-9))==a*b)
            break;
        else
            Qant = Q;
        end
    end
    Q = Q/max(max(Q)); % Normalize
    Q = round(Q*100);
end

function [pi] = policy(Q,start,goal)
    pi = start;
    while(pi(end) ~= goal)
        a = find(Q(pi(end),:) == max(Q(pi(end),:)));
        pi = [pi,a(randi(length(a)))];
    end
end