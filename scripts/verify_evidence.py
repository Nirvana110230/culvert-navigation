"""校验 G3 源码指纹、归档断言、逐例留出统计；在工作区根目录以 Python 3 运行。"""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs" / "evidence"


def load(path):
    return json.loads(path.read_bytes())


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(condition, message):
    if not condition:
        raise ValueError(message)


def check_tree(directory, manifest):
    expected = load(EVIDENCE / manifest)
    actual = {
        p.relative_to(directory).as_posix(): sha256(p)
        for p in directory.rglob("*") if p.is_file()
    }
    check(actual == expected, manifest)
    return len(actual)


def main():
    result = {}
    for directory, name in [("isolated/v3", "v3-source-sha256.json"),
                            ("third_party/small_gicp", "small-gicp-build-sources.json")]:
        result[directory] = check_tree(ROOT / directory, name)

    packages = load(EVIDENCE / "board-package-sources.json")
    for rel, record in packages.items():
        data = (ROOT / rel).read_bytes()
        if record["checkout_comparison"] == "documentation_links_updated":
            check(Path(rel).name == "README.md", rel)
            continue
        normalized = data.replace(b"\r\n", b"\n")
        check(record["board_sha256"] in (hashlib.sha256(data).hexdigest(),
                                        hashlib.sha256(normalized).hexdigest()), rel)

    identity = load(ROOT / "VERSION-IDENTITIES.json")
    patch = ROOT / "patches/fast_livo2_culvert_source.patch"
    check(sha256(patch) == identity["isolated_v3"]["patch_sha256"], "v3 patch")
    check(load(EVIDENCE / "r2-freeze.json")["binary_sha256"] ==
          identity["worker"]["installed_arm64_sha256"], "worker hash")

    paths = {"ros-board/assertions.json": "r2-ros-first.json",
             "ros-board-retest/assertions.json": "r2-ros-retest.json",
             "r1-regression/synthetic.json": "r1-ros.json",
             "soak/end.json": "r2-soak.json", "evaluation/freeze.json": "r2-freeze.json"}
    hashes = load(EVIDENCE / "board-evidence-sha256.json")
    for original, saved in paths.items():
        check(sha256(EVIDENCE / saved) == hashes[original], original)
    for name, field, count in [("r1-ros.json", "pass", 25), ("r2-ros-retest.json", "passed", 50)]:
        outcomes = load(EVIDENCE / name)
        check(len(outcomes) == count and all(row[field] for row in outcomes), name)
        result[name] = count
    cases = ROOT / "benchmarks/r2/results/cases.csv"
    check(sha256(cases) == identity["evidence_sha256"]["benchmarks/r2/results/cases.csv"], "case table")
    with cases.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for group in identity["historical_holdout"]:
        backend = group["backend"]
        counter = Counter(row["outcome"] for row in rows
                          if row["split"] == "holdout" and row["backend"] == backend)
        check(sum(counter.values()) == group["n"], backend)
        for measured, stored in [("correct_accept", "correct_accepted"),
                                 ("false_accept", "false_accepted"), ("correct_reject", "correct_rejected")]:
            check(counter[measured] == group[stored], backend + "/" + measured)
        result[backend] = dict(counter)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
