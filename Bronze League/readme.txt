TOTAL time so far : 6 hours.
	1 hour for the wood league
	5 for Bronze, not counting the time waiting for battles
	a few minutes Friday morning, 21H20-00H30 friday, 9H40-11H40 and 13H15-14H15 sunday, and parameter tweaking after adjusting
	

Ranking : 1st over ~35.000

Key concepts
-Keeping on track (keep_track) by nudging the target
-handle_avoidance : very simple avoidance (handle_avoidance, can't implement RVO in such limited time for now)
-overshoot detection (inlined in code)
-check_can_boost when aligned

Problems
-I tried to stick to integers where possible, but it is absurd. I should have used only VectorF_t instead of Vector_t
-in the Overshoot part, line 223, I defined a constant that I do not use and the 'for' is not synchronized to it