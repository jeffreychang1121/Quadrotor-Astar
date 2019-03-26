function effort = consideration(cur,candidates,cur_cost)

% goal: 3 by 1 matrix (xyz coordinate)
% cur: 3 by 1 matrix (xyz coordinate)
% candidates: 3 by 1 matrix (xyz coordinate)

effort = cur_cost + norm((candidates - cur));

end