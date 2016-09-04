function data = readData( tline )
  data = strsplit(tline(6:end));
  data = data(~cellfun('isempty',data)); %remove empty elements     
end

