import numpy as np
import json


# -------------------------------------------------------------------------------------------


# Generate an numpy array of targets where no agents are placed

def generate_targets(size, n_targets, agents_position):
	targets = []

	while len(targets) < n_targets:
		# (2,1) numpy array of random floating values with range size-2 starting from 2 to leave 2 meters free in front of the agent
		value = np.random.random((2, 1)) * (size-2) + 2  

		cond1 = not (value == agents_position).all()
		cond2 = False not in [not (value == t).all() for t in targets]

		if cond1 and cond2:
			targets.append(value.tolist())
	return targets


# -------------------------------------------------------------------------------------------
# --------------------------------- MAIN PROGRAM --------------------------------------------
# -------------------------------------------------------------------------------------------


size = 15  # size of the grid
n_targets_max = 10  # number of targets to collect
agent_position = np.array([[0.], [0.]])  # agent at the origin of the grid
n_simulations = 30

if __name__ == "__main__":

	targets_scenarios = []

	for n_targets in range(2, n_targets_max+1):
		for i in range(0, n_simulations):

			targets = generate_targets(size, n_targets, agent_position)  # generate targets
			targets_scenarios.append(targets)

	# Write down
	with open('scenarios.txt', 'w') as f: 
		f.write(json.dumps(targets_scenarios))
