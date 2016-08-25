function original_plan = readOriginalPlan(tline)
  original_plan = strsplit(tline(6:end))
  for i = 1:1:length(original_plan)
       original_plan{i} = original_plan{i}(2:end)
  end
  original_plan = original_plan(~cellfun('isempty',original_plan)) %remove empty elements     
end

