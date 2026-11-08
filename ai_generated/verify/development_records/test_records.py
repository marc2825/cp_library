"""Independent fixtures for provenance, mappings, stale evidence and build modes."""
import copy
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

from tools.cp_tools import catalog, provenance, suite
from tools.cp_tools.repository import closure, digest, include_graph, read_json, write_json


class Repository(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(prefix="cp-records-test-")
        self.root = Path(self.folder.name)
        self.addCleanup(self.folder.cleanup)
        self.put("lib/a.hpp", '#pragma once\n#include "b.hpp"\n')
        self.put("lib/b.hpp", "#pragma once\n")
        self.put("verify/sample/unit.cpp", '#include "lib/a.hpp"\nint main(){}\n')
        self.put("verify/sample/problem.test.cpp", '#define PROBLEM "https://judge.example/problem/a"\n#include "lib/a.hpp"\nint main(){}\n')
        self.put("verify/run_sample.py", 'import sys\nprint("ok")\n')
        write_json(self.root / "verification/provenance.json", {"schema_version": 1, "entries": []})
        write_json(self.root / "verification/accepted.json", {"schema_version": 1, "submissions": []})

    def put(self, name, text):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    def row(self, data, name="lib/a.hpp"):
        return next(r for r in data["features"] if r["path"] == name)

    def record_provenance(self):
        self.put("license.txt", "sample license\n")
        record = {"id": "sample", "kind": "adapted", "sources": ["https://example.com/source"],
                  "imported_revision": "a" * 40, "changes": "namespace", "local_sha256": {"lib/a.hpp": digest(self.root / "lib/a.hpp")},
                  "license": {"id": "CC0-1.0", "evidence": "https://example.com/license", "file": "license.txt", "sha256": digest(self.root / "license.txt")}}
        write_json(self.root / "verification/provenance.json", {"schema_version": 1, "entries": [record]})
        return record

    def receipt(self, **extra):
        record = {"schema_version": 1, "runner": "sample", "profile": "standard", "status": "passed", "snapshot": catalog.snapshot_id(self.root)}
        record.update(extra)
        write_json(self.root / "verification/evidence/sample/standard.json", record)

    def test_provenance_hash_and_license_changes(self):
        self.record_provenance()
        self.assertEqual(provenance.inventory(self.root)["files"][0]["record"], "sample")
        self.put("license.txt", "changed")
        with self.assertRaisesRegex(ValueError, "license text changed"):
            provenance.inventory(self.root)
        self.put("license.txt", "sample license\n")
        self.put("lib/a.hpp", "// changed\n")
        with self.assertRaisesRegex(ValueError, "hash changed"):
            provenance.inventory(self.root)

    def test_provenance_unknown_commit_duplicates_and_new_attribution(self):
        record = self.record_provenance()
        record["imported_revision"] = None
        write_json(self.root / "verification/provenance.json", {"schema_version": 1, "entries": [record]})
        with self.assertRaisesRegex(ValueError, "unknown revision"):
            provenance.inventory(self.root)
        record["revision_note"] = "historical unknown"
        write_json(self.root / "verification/provenance.json", {"schema_version": 1, "entries": [record]})
        self.assertIsNone(provenance.inventory(self.root)["entries"][0]["imported_revision"])
        write_json(self.root / "verification/provenance.json", {"schema_version": 1, "entries": [record, record]})
        with self.assertRaisesRegex(ValueError, "duplicate"):
            provenance.inventory(self.root)
        write_json(self.root / "verification/provenance.json", {"schema_version": 1, "entries": [record]})
        self.put("lib/new.hpp", "// Adapted from example\n")
        with self.assertRaisesRegex(ValueError, "unregistered"):
            provenance.inventory(self.root)

    def test_paths_cannot_escape_repository(self):
        record = self.record_provenance()
        record["local_sha256"] = {"../outside.hpp": "0" * 64}
        write_json(self.root / "verification/provenance.json", {"schema_version": 1, "entries": [record]})
        with self.assertRaisesRegex(ValueError, "outside repository"):
            provenance.inventory(self.root)

    def test_graph_comments_raw_strings_conditions_and_cycles(self):
        self.put("lib/a.hpp", '''// #include "missing.hpp"
const char* s=R"(#include "missing.hpp")";
#if 0
#include "b.hpp"
#endif
''')
        self.put("lib/b.hpp", '#include "a.hpp"\n')
        graph = include_graph(self.root)
        self.assertEqual(graph["lib/a.hpp"], ["lib/b.hpp"])
        self.assertEqual(closure(graph, "lib/a.hpp"), ["lib/a.hpp", "lib/b.hpp"])

    def test_mapping_does_not_claim_execution_or_ac(self):
        data = catalog.collect(self.root)
        row = self.row(data)
        self.assertEqual(row["status"], "unverified")
        self.assertEqual(row["problems"], ["https://judge.example/problem/a"])
        self.assertEqual(row["runners"], ["sample"])
        self.assertEqual(len(row["tests"]), 2)
        self.assertEqual(self.row(data, "lib/b.hpp")["tests"], row["tests"])

    def test_success_failure_stale_and_added_source(self):
        self.receipt()
        self.assertEqual(self.row(catalog.collect(self.root))["status"], "local-only")
        self.receipt(status="failed")
        self.assertEqual(self.row(catalog.collect(self.root))["status"], "unverified")
        self.receipt()
        self.put("lib/new.hpp", "// new\n")
        data = catalog.collect(self.root)
        self.assertFalse(data["attempts"][0]["current"])
        self.assertEqual(self.row(data)["status"], "unverified")
        self.assertEqual(self.row(data, "lib/new.hpp")["status"], "unverified")

    def test_test_edits_invalidate_receipts_but_evidence_does_not(self):
        self.receipt()
        before = catalog.snapshot_id(self.root)
        self.receipt(profile="stdlib-debug")
        self.assertEqual(catalog.snapshot_id(self.root), before)
        self.put("verify/sample/unit.cpp", "int main(){return 1;}\n")
        self.assertEqual(self.row(catalog.collect(self.root))["status"], "unverified")

    def test_invalid_receipts_are_errors(self):
        self.receipt(profile="invented")
        with self.assertRaisesRegex(ValueError, "profile"):
            catalog.collect(self.root)

    def test_manifest_only_mapping_and_missing_header(self):
        self.put("lib/manifest_only.hpp", "#pragma once\n")
        write_json(self.root / "verify/sample/manifest.json", {"algorithms": [{"header": "lib/manifest_only.hpp", "title": "Example"}]})
        data = catalog.collect(self.root)
        self.assertEqual(self.row(data, "lib/manifest_only.hpp")["runners"], ["sample"])
        self.assertEqual(self.row(data, "lib/manifest_only.hpp")["tests"], [])
        self.put("verify/sample/manifest.json", '{"algorithms":[{"header":"missing.hpp"}]}')
        with self.assertRaises(ValueError):
            catalog.collect(self.root)

    def test_accepted_submission_needs_complete_current_sources(self):
        test = "verify/sample/problem.test.cpp"
        graph = include_graph(self.root)
        record = {"test": test, "problem": "https://judge.example/problem/a", "verdict": "AC", "submission_url": "https://judge.example/submission/123", "reviewed_at": "2026-10-20", "sources": {p: digest(self.root / p) for p in closure(graph, test)}}
        write_json(self.root / "verification/accepted.json", {"schema_version": 1, "submissions": [record]})
        self.assertEqual(self.row(catalog.collect(self.root))["status"], "online-ac")
        self.put("lib/b.hpp", "// changed dependency\n")
        self.assertEqual(self.row(catalog.collect(self.root))["status"], "unverified")
        record["problem"] = "https://judge.example/problem/wrong"
        write_json(self.root / "verification/accepted.json", {"schema_version": 1, "submissions": [record]})
        with self.assertRaisesRegex(ValueError, "PROBLEM"):
            catalog.collect(self.root)

    def test_html_escapes_titles_and_uses_no_remote_scripts(self):
        write_json(self.root / "verify/sample/manifest.json", {"algorithms": [{"header": "lib/a.hpp", "title": "<script>alert(1)</script>"}]})
        page = catalog.render_html(catalog.collect(self.root))
        self.assertIn("&lt;script&gt;alert(1)&lt;/script&gt;", page)
        self.assertNotIn('<script src=', page)
        self.assertIn('id="query"', page)

    def test_suite_pass_fail_timeout_and_replaces_old_pass(self):
        compiler = ["g++"]
        kwargs = dict(root=self.root, output=self.root / "build/suite", timeout=5)
        self.assertTrue(suite.run_one("sample", "verify/run_sample.py", compiler, **kwargs))
        self.assertEqual(self.row(catalog.collect(self.root))["status"], "local-only")
        self.put("verify/run_sample.py", "raise SystemExit(2)\n")
        self.assertFalse(suite.run_one("sample", "verify/run_sample.py", compiler, **kwargs))
        receipt = read_json(self.root / "verification/evidence/sample/standard.json")
        self.assertEqual(receipt["returncode"], 2)
        self.assertEqual(receipt["status"], "failed")
        self.put("verify/run_sample.py", "import time\ntime.sleep(5)\n")
        kwargs["timeout"] = .1
        self.assertFalse(suite.run_one("sample", "verify/run_sample.py", compiler, **kwargs))
        self.assertTrue(read_json(self.root / "verification/evidence/sample/standard.json")["timeout"])


class BuildModes(unittest.TestCase):
    def test_compiler_profile_owns_macros(self):
        self.assertEqual(suite.compiler_command("ccache g++", True), ["ccache", "g++", *suite.DEBUG_FLAGS])
        self.assertEqual(suite.compiler_command("g++", False), ["g++"])
        with self.assertRaises(ValueError):
            suite.compiler_command("", True)
        with self.assertRaises(ValueError):
            suite.compiler_command("g++ -U_GLIBCXX_DEBUG", True)

    def test_libstdcpp_detects_invalid_iterators_even_with_ndebug(self):
        compiler = suite.compiler_command(os.environ.get("CP_TEST_CXX", "g++"), True)
        suite.preflight(compiler, True)
        with tempfile.TemporaryDirectory(prefix="cp-debug-negative-") as td:
            path = Path(td)
            # Two translation units exchange a std::vector with the same ABI.
            (path / "a.cpp").write_text('#include <vector>\nstd::vector<int> values(){return {3,5};}\n')
            (path / "b.cpp").write_text('#include <vector>\nstd::vector<int> values();int main(){auto v=values();return v[0]+v[1]!=8;}\n')
            subprocess.run([*compiler, "-std=c++17", str(path / "a.cpp"), str(path / "b.cpp"), "-o", str(path / "multi")], check=True, capture_output=True)
            subprocess.run([str(path / "multi")], check=True, capture_output=True)
            (path / "bad.cpp").write_text('#include <vector>\nint main(){std::vector<int> v{1};auto it=v.begin();v.reserve(100);return *it;}\n')
            for flags in ([], ["-DNDEBUG"]):
                subprocess.run([*compiler, "-std=c++17", *flags, str(path / "bad.cpp"), "-o", str(path / "bad")], check=True, capture_output=True)
                # Prevent a core file for this deliberate negative test.
                def no_core():
                    import resource
                    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
                result = subprocess.run([str(path / "bad")], capture_output=True,
                                        preexec_fn=no_core if os.name == "posix" else None)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(b"singular iterator", result.stderr)


if __name__ == "__main__":
    unittest.main()
