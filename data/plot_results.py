import json
from matplotlib import pyplot as plt
import numpy as np

    
with open("scenarios.txt", 'r') as f: targets_scenarios = json.loads(f.read())

nb_scenarios = len(targets_scenarios)
max_targets = len(targets_scenarios[-1])
nb_sublists = max_targets - 1
simulations_per_scenarios = int(nb_scenarios/nb_sublists)

with open('mcts_results.txt', 'r') as f: res_mcts = json.loads(f.read())
with open('aco_results.txt', 'r') as f: res_aco = json.loads(f.read())

L_mcts, L_aco = [], []

for i in range(2, max_targets+1):
    L_mcts_ith_scenario, L_aco_ith_scenario = [], []

    for j in range(nb_scenarios):
        if len(targets_scenarios[j]) == i:
            L_mcts_ith_scenario.append(res_mcts[j])
            L_aco_ith_scenario.append(res_aco[j])

    L_mcts.append(L_mcts_ith_scenario)
    L_aco.append(L_aco_ith_scenario)

for i in range(len(L_mcts)):
    L_mcts[i] = np.mean(L_mcts[i])
    L_aco[i] = np.mean(L_aco[i])

plt.figure(figsize=(20, 10))
# plt.ylim(25, 75)

font = {'family': 'serif', 'color':  'black', 'weight': 'normal', 'size': 16}
        
plt.title("Travelled distance VS. Number of waypoints", fontdict=font)
plt.xlabel("Number of waypoints", fontdict=font)
plt.ylabel("Average travelled distance across " + str(simulations_per_scenarios) + " simulations", fontdict=font)

plt.plot(np.arange(2, max_targets+1, 1), L_mcts, label="MCTS", color='green', marker='o')
plt.plot(np.arange(2, max_targets+1, 1), L_aco, label="ACO", color='blue', marker='o')

# Confidence interval across all simulations
c1 = 1.96 * np.std(L_mcts)/np.mean(L_mcts)  # MCTS
c2 = 1.96 * np.std(L_aco)/np.mean(L_aco)  # ACO

plt.fill_between(np.arange(2, max_targets+1, 1), (L_mcts-c1), (L_mcts+c1), color='green', alpha=.3)
plt.fill_between(np.arange(2, max_targets+1, 1), (L_aco-c2), (L_aco+c2), color='blue', alpha=.3)

plt.legend(fontsize="xx-large")
plt.legend(prop={'family': 'serif', 'size': 16})

plt.savefig('MCTS_VS_ACO.png')
# plt.close()
plt.show()
