#!/usr/bin/env python3
import re
import os
import glob

max_high_data = []
total_data = []

for result_file in sorted(glob.glob("../*/result.txt")):
    test_name = os.path.basename(os.path.dirname(result_file))
    expect_file = result_file.replace("result.txt", "expect.txt")

    if not os.path.exists(expect_file):
        continue

    with open(result_file) as f:
        result_content = f.read()
    with open(expect_file) as f:
        expect_content = f.read()

    for line in result_content.split("\n"):
        if "Step" in line and "Max High=" in line:
            m = re.search(r"(Step\w+):.*Max High=(\d+)us Total High=(\d+)us", line)
            if m:
                step_name = m.group(1)
                max_high = int(m.group(2))
                total_high = int(m.group(3))

                exp_line = [
                    l
                    for l in expect_content.split("\n")
                    if step_name in l and "Max High" in l
                ]
                if exp_line:
                    em = re.search(
                        r"Max High<=(\d+)us Total High<=(\d+)us", exp_line[0]
                    )
                    if em:
                        exp_max = int(em.group(1))
                        exp_total = int(em.group(2))
                        max_high_data.append(
                            {
                                "test": test_name.replace("test_sd_04_timing_", ""),
                                "step": step_name,
                                "result": max_high,
                                "expect": exp_max,
                            }
                        )
                        total_data.append(
                            {
                                "test": test_name.replace("test_sd_04_timing_", ""),
                                "step": step_name,
                                "result": total_high,
                                "expect": exp_total,
                            }
                        )

print("\n### Max High Time (us): Actual vs Expected\n")
print("| Test | Step | Actual | Expected |")
print("|------|------|--------|----------|")
for d in sorted(max_high_data, key=lambda x: (x["test"], x["step"])):
    print(f"| {d['test']} | {d['step']} | {d['result']} | {d['expect']} |")

print("\n### Total High Time (us): Actual vs Expected\n")
print("| Test | Step | Actual | Expected |")
print("|------|------|--------|----------|")
for d in sorted(total_data, key=lambda x: (x["test"], x["step"])):
    print(f"| {d['test']} | {d['step']} | {d['result']} | {d['expect']} |")

with open("scatter_max_high.dat", "w") as f:
    f.write("# expected actual label\n")
    for d in max_high_data:
        f.write(f'{d["expect"]} {d["result"]} "{d["test"]}:{d["step"]}"\n')

with open("scatter_total.dat", "w") as f:
    f.write("# expected actual label\n")
    for d in total_data:
        f.write(f'{d["expect"]} {d["result"]} "{d["test"]}:{d["step"]}"\n')

print("\n\nData files: scatter_max_high.dat, scatter_total.dat")
