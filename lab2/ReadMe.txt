The lab2.py contains code for bfs, dfs, A*, UCS also called branch and bound

A* : Global options from the fringe will be considered. add sorting will be done via cummulative distance & overall heuristic.

UCS : Considers the cummulative minimum from all global fringe options available.

Hill climbing - Allows bactracking but a true hill climbing doesn't. Ideally it should delete the neighbours which ere not selected at ith level.

Beam Search - this one doesn't work as expected but when I send result = [] in all cases then the result expected is what I actually send
but some cases do pass. However I think my logic is correct.


Other searches :

Best first search. 

Best First Search vs Hill Climbing : One of them is global
