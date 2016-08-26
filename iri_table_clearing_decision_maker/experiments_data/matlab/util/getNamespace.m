function s = getNamespace(tline)
   s = '';
    for i=1:1:length(tline)
        if( tline(i) == ':')
            if i > 1 
                s = tline(1:i-1);
                break
            end
        end
    end
end

