function [ ] = assignment1()
% This code solves and present the solution from reaching the oposite
% corner in a maze in an optimal way using the A* algorithm.

% "corner" is the starting position: 0 := Northwest, 2 := Northeast, 
%   3 = Southwest and 4:= Southeast is determined by the studentNumber
%   module with 4
    
    % maze to be solved
    maze = [0,0,0,1,0,0,0,0,0,0,0,0; 0,0,0,1,0,0,1,1,1,1,1,1; ...
        1,0,0,1,0,0,0,0,0,0,0,0; 0,0,0,0,0,0,0,0,0,0,0,0; ...
        0,1,1,0,0,1,0,0,1,1,0,1; 0,0,1,0,0,1,0,0,1,0,0,0; ...
        0,0,1,0,0,0,0,0,1,0,0,0; 0,0,1,0,0,0,0,0,1,0,0,0; ...
        0,0,1,1,1,1,1,0,1,0,1,1; 1,0,0,0,0,0,0,0,1,0,0,0; ...
        0,0,0,1,1,1,1,0,1,0,0,0; 0,0,0,1,0,0,0,0,0,0,0,0];
    % choose the right starting corner
    studentNumber = 502029;
    corner = mod(studentNumber,4); % 0,1,2 and 3
    [a,b] = size(maze);
    % generate the start and goal point based on corner
    switch corner
        case 0
            start = [1,1];
            goal = [a,b];
        case 1
            start = [1,b];
            goal = [a,1];
        case 2
            start = [a,1];
            goal = [1,b];
        case 3
            start = [a,b];
            goal = [1,1];
    end
    % find the path using the basic version
    tic;
    [S1,n1] = solveMaze1(maze,start,goal);
    t1 = toc;
    % find the path using dynamic programing
    tic;
    [S2,n2] = solveMaze2(maze,start,goal);
    t2 = toc;
    % Graphics:
    figure; imshow(S1,'InitialMagnification',3000);
    title(['Basic Version, ',num2str(n1),...
        ' nodes expanded, time = ',num2str(t1),'s'])
    figure; imshow(S2,'InitialMagnification',3000);
    title(['Dynamic Programming, ',num2str(n2),...
        ' nodes expanded, time = ',num2str(t2),'s'])
end

function [ S,n ] = solveMaze1(maze,start,goal)
% solveMaze: find the optimal route between two points in a maze.
% Basic version more than one search node for each state. 
    S = maze;
    current = start;
    n = 0;
    % candidates = [position,previousPosition(n),cost,estimatedCost]
    candidates = [current(1),current(2),n,0,norm(current-goal)];
    expanded = [];
    while (true)
        % Find the most promising node and expand it:
        index = find(candidates(:,5)==min(candidates(:,5)),1);
        if n == 0
            new = expandNode(maze,candidates(index,1),...
                candidates(index,2),0,0);
        else
            new = expandNode(maze,candidates(index,1),...
                candidates(index,2),expanded(n,1),expanded(n,2));
        end
        % Calculate and store values for new candidates:
        n = n + 1;
        for i = 1:size(new,1)
            new(i,3) = n; % previousPosition
            new(i,4) = candidates(index,4) + 1; % cost
            new(i,5) = new(i,4) + norm(new(i,1:2)-goal); % estimadedCost
            S(new(i,1),new(i,2)) = 0.2; % Graphics for expanded node
        end
        current = candidates(index,1:2); % New current position.
        expanded = [expanded;candidates(index,1:3)]; % Save expanded node
        candidates(index,:) = []; % Erase expanded node from candidates
        candidates = [candidates;new]; % Add new candidates to the list
        if sum(current==goal)==2; break; end
    end
    
    % Graphics for optimal path:
    c = expanded(end,3);
    while c~=0
        S(expanded(c,1),expanded(c,2)) = 0.4;
        c = expanded(c,3);
    end
    S(start(1),start(2)) = 0.6;
    S(goal(1),goal(2)) = 0.8;
end

function [ S,n ] = solveMaze2(maze,start,goal)
% solveMaze: find the optimal route between two points in a maze.
% Second version, redundant paths eliminated by storing at most one search 
% per node for each state, using the dynamic programming principle. 
    S = maze;
    [a,b] = size(S);
    cost = inf(a,b);
    estimatedCost = inf(a,b);
    previous = zeros(a,b,2);
    cost(start(1),start(2)) = 0;
    estimatedCost(start(1),start(2)) = norm(start-goal);
    n = 0;
    while (true)
        [px,py] = ind2sub([a,b],...
            find(estimatedCost==min(min(estimatedCost)),1)); % Current
        new = expandNode(maze,px,py,previous(px,py,1),previous(px,py,2));
        for i = 1:size(new,1)
            if cost(new(i,1),new(i,2)) > cost(px,py) + 1
                previous(new(i,1),new(i,2),1:2) = [px,py]; % previousPosition
                cost(new(i,1),new(i,2)) = cost(px,py) + 1; % cost
                estimatedCost(new(i,1),new(i,2)) = cost(new(i,1),...
                    new(i,2)) + norm(new(i,:)-goal); % estimadedCost
                S(new(i,1),new(i,2)) = 0.2; % Graphics for expanded node
            end
        end
        estimatedCost(px,py) = Inf;
        n = n + 1;
        if sum([px,py]==goal)==2; break; end
    end
    % Graphics:
    c = goal;
    while previous(c(1),c(2),1)~=0
        S(c(1),c(2)) = 0.4;
        c = previous(c(1),c(2),:);
    end
    S(start(1),start(2)) = 0.6;
    S(goal(1),goal(2)) = 0.8;
end

function [v] = expandNode(maze,x,y,xa,ya)
    v = [];
    [a,b] = size(maze);
    if x-1>0&&(xa~=x-1||ya~=y); if maze(x-1,y)~=1; v = [v;[x-1,y]]; end, end
    if y-1>0&&(xa~=x||ya~=y-1); if maze(x,y-1)~=1; v = [v;[x,y-1]]; end, end
    if x+1<=a&&(xa~=x+1||ya~=y); if maze(x+1,y)~=1; v = [v;[x+1,y]]; end, end
    if y+1<=b&&(xa~=x||ya~=y+1); if maze(x,y+1)~=1; v = [v;[x,y+1]]; end, end
end