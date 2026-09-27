import json, os, subprocess, sys, glob

problems = sorted(glob.glob('problems/*.json'))

passed, failed = 0, 0
failures = []

for problem_path in problems:
    stem = os.path.basename(problem_path).replace('.json', '')
    solution_path = os.path.join('solutions_mine', stem + '.json')

    if not os.path.exists(solution_path):
        continue

    result = subprocess.run(
        [sys.executable, 'displib_verify.py', problem_path, solution_path],
        capture_output=True, text=True
    )
    output = result.stdout + result.stderr

    # 1. decide ok: does "is feasible" appear in output?
    # 2. if ok, passed += 1; else failed += 1 and append (stem, output) to failures
    not_ok = "Error" in output

    print(f"{stem:26s} {'OK' if not not_ok else 'FAIL'}")
    if not_ok:
        failed += 1
        failures.append((stem, output))
    else:
        passed += 1

print(f"\npassed {passed}, failed {failed}, of {passed + failed}")

for stem, output in failures[:5]:
    print(f"\n=== {stem}")
    for line in output.splitlines():
        if 'Error' in line or 'event' in line:
            print("   ", line.strip())