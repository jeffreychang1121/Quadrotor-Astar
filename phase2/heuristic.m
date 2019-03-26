function effort = heuristic(goal,candidates)

% goal: 3 by 1 matrix (xyz coordinate)
% candidates: 3 by 1 matrix (sub coordinate)

effort = norm((goal - candidates));

end