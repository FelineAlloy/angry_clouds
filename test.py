import random
from test_case_gen import generate_test
from main import solve
from grader import grade
from visualizer import visualize


MAX_M = 10 #70          # Max grid width
MAX_N = 10 #70          # Max grid height
MAX_FN = 1 #5000       # Max number of flows
MAX_T = 100         # Max simulation time (seconds)
MAX_B = 10 #1000.0      # Max peak bandwidth per UAV (Mbps)
MAX_Q_TOTAL = 100 #3000  # Max total traffic volume per flow (Mbits)

M = random.randint(2, MAX_M)             # grid width
N = random.randint(2, MAX_N)             # grid height
FN = random.randint(1, MAX_FN)           # number of flows
T = random.randint(2, MAX_T)             # simulation duration (seconds)

generate_test(M, N, FN, T)
solve()
grade()
visualize()

