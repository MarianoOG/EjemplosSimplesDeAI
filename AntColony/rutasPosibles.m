function [rutasP] = rutasPosibles(actual,anterior,rutas)
    a = find(rutas == actual);
    % rutasP = zeros(length(a)-1,1); % Asume que las rutas no se repiten
    
    n = size(rutas,1);
    j = 1;
    for i = 1:size(a)
        if a(i)<=n
            b = a(i) + n;
        else
            b = a(i) - n;
            a(i) = a(i) - n;
        end
        if(rutas(b) ~= anterior)
            rutasP(j) = a(i);
            j = j + 1;
        end
    end
    
end