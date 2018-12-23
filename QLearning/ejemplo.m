function [] = ejemplo()
    clc;
    % Variables:
    gamma = 0.8;
    
    start = 3;
    goal = 6;
    R = [-1,-1,-1,-1,0,-1;
        -1,-1,-1,0,-1,100;
        -1,-1,-1,0,-1,-1;
        -1,0,0,-1,0,-1;
        0,-1,-1,0,-1,100;
        -1,0,-1,-1,0,100];
    
    % Calculating Q values and policy:
    [Q,n] = qlearning(R,gamma);
    pi = policy(Q,start,goal);
    
    % Display answer:
    disp(['Convergence on: ',num2str(n),' episodes']);
    disp('Q: '); disp(Q);
    disp(['Policy: ',num2str(pi)]);
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

function [pi] = policy( Q, start, goal )
    pi = start;
    while(pi(end) ~= goal)
        a = find(Q(pi(end),:) == max(Q(pi(end),:)));
        pi = [pi,a(randi(length(a)))];
    end
end