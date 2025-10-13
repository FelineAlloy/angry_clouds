import subprocess
import sys
from pathlib import Path

# ===============================
# CONFIGURABLE MAX CONSTANTS
# ===============================
MAX_M = 70 #70          # Max grid width
MAX_N = 70 #70          # Max grid height
MAX_FN = 5000 #5000       # Max number of flows
MAX_T = 500         # Max simulation time (seconds)
MAX_B = 10 #1000.0      # Max peak bandwidth per UAV (Mbps)
MAX_Q_TOTAL = 100 #3000  # Max total traffic volume per flow (Mbits)
# ===============================
# ===============================

BASE = Path(__file__).parent.resolve()
SUBMISSION = BASE / "submission.py"
MAIN = BASE / "main.py"
GRADER = BASE / "grader.py"
TEST_GEN = BASE / "test_case_gen.py"

def generate_case(case_path):
    print(f"Generating test case -> {case_path}")
    subprocess.run([sys.executable, str(TEST_GEN)])

def run_solution(script_path, input_path, output_path):
    print(f"Running {script_path.name} ...")
    with open(input_path, "r") as inp, open(output_path, "w") as out:
        subprocess.run([sys.executable, str(script_path)], stdin=inp, stdout=out, check=True)

def grade_case(input_path, output_path):
    print(f"Grading {output_path}")
    subprocess.run([sys.executable, str(GRADER), str(input_path), str(output_path)], check=True)

def summarize_case_params(case_path):
    with open(case_path, "r", encoding="utf-8") as f:
        header = f.readline().split()
        if len(header) < 4:
            print("Invalid header line.")
            return
        M, N, FN, T = map(int, header[:4])

        maxB = 0.0
        for _ in range(M * N):
            parts = f.readline().split()
            if len(parts) >= 3:
                maxB = max(maxB, float(parts[2]))

    print(f"Grid: {M}×{N}")
    print(f"Flows: {FN}")
    print(f"Time duration: {T}s")
    print(f"Max B: {MAX_B} Mbps, Max Flow Size: {MAX_Q_TOTAL} Mbits")

def main():
    case_path = "in1.in" 
    sub_out = "out.out" 

    generate_case(case_path)

    run_solution(SUBMISSION, case_path, sub_out)

    print("\n[=] Submission result:")
    grade_case(case_path, sub_out)

    print("")
    summarize_case_params(case_path)
    print("")

if __name__ == "__main__":
    main()
