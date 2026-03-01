#!/usr/bin/env python3
import re
import os
import glob

timing_data = []

for result_file in sorted(glob.glob("*/result.txt")):
    test_name = os.path.dirname(result_file)
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
                        timing_data.append(
                            {
                                "test": test_name,
                                "metric": f"{step_name}_Max_High_us",
                                "result": max_high,
                                "expect": exp_max,
                            }
                        )
                        timing_data.append(
                            {
                                "test": test_name,
                                "metric": f"{step_name}_Total_High_us",
                                "result": total_high,
                                "expect": exp_total,
                            }
                        )

        elif "Time in" in line and "max=" in line:
            m = re.search(r"Time in (\w+)\s+max=(\d+)\s*us,\s*total=(\d+)", line)
            if m:
                isr_name = m.group(1)
                max_val = int(m.group(2))
                total_val = int(m.group(3))

                exp_line = [
                    l for l in expect_content.split("\n") if f"Time in {isr_name}" in l
                ]
                if exp_line:
                    em = re.search(r"max<=(\d+)\s*us,\s*total<=(\d+)", exp_line[0])
                    if em:
                        exp_max = int(em.group(1))
                        exp_total = int(em.group(2))
                        timing_data.append(
                            {
                                "test": test_name,
                                "metric": f"{isr_name}_Max_us",
                                "result": max_val,
                                "expect": exp_max,
                            }
                        )
                        timing_data.append(
                            {
                                "test": test_name,
                                "metric": f"{isr_name}_Total_us",
                                "result": total_val,
                                "expect": exp_total,
                            }
                        )

seen = set()
unique_data = []
for d in timing_data:
    key = (d["test"], d["metric"])
    if key not in seen:
        seen.add(key)
        unique_data.append(d)

print("=" * 90)
print(f"{'Test':<30} {'Metric':<22} {'Result':>10} {'Expect':>10} {'Util%':>8}")
print("=" * 90)

for d in sorted(unique_data, key=lambda x: (x["test"], x["metric"])):
    util = (d["result"] / d["expect"]) * 100 if d["expect"] > 0 else 0
    status = "OK" if d["result"] <= d["expect"] else "FAIL"
    print(
        f"{d['test']:<30} {d['metric']:<22} {d['result']:>10} {d['expect']:>10} {util:>7.1f}% {status}"
    )

with open("timing_comparison.dat", "w") as f:
    f.write("# Test Metric Result Expect Utilization\n")
    for d in sorted(unique_data, key=lambda x: (x["test"], x["metric"])):
        util = (d["result"] / d["expect"]) * 100 if d["expect"] > 0 else 0
        f.write(
            f'"{d["test"]}" "{d["metric"]}" {d["result"]} {d["expect"]} {util:.1f}\n'
        )

print("\nData written to timing_comparison.dat")
