"""The vendored X-31 port is byte-identical to its upstream source.

`python/x31/` is GPLv2 third-party source, Copyright (C) 2016 Hsin-Yi Kang,
carried in from `Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft` at
commit `c03cc39`. Its measured parity against MATLAB holds on that exact form,
so an edit here does not make the port better; it makes the parity evidence
describe code that no longer exists. These tests are the mechanical guard
against that, and against losing the attribution.

The one intended difference is the module docstring: each file keeps the
upstream first line verbatim and adds the provenance note below it. So each
file is compared as everything *after* its module docstring, hashed against the
upstream file hashed after *its* first line. `UPSTREAM_DIGESTS` was computed
from the upstream checkout at vendoring time; regenerate it only when
deliberately re-vendoring a newer upstream commit, and say so in the commit
message, because that is exactly the change that voids the parity evidence.
"""

import hashlib
import sys
import unittest
from pathlib import Path

_PY = Path(__file__).resolve().parents[1]
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

import x31_numpy_compat  # noqa: F401,E402  restores numpy's removed short trig aliases

import x31  # noqa: E402

_X31_DIR = _PY / "x31"
_GPL_LINE = "X-31 Python port. Copyright (C) 2016 Hsin-Yi Kang. GPLv2."
_UPSTREAM = "Dynamic-And-Control-Model-Of-SuperManeuverable-Aircraft"
_UPSTREAM_COMMIT = "c03cc39"

UPSTREAM_DIGESTS = {
    "__init__.py": "01ba4719c80b6fe911b091a7c05124b64eeece964e09c058ef8f9805daca546b",
    "actuators.py": "c2fb6e414e8ebbd9d92ed23cda84449e671fb2177c1ef5c691c2c50c8b838fcc",
    "aero.py": "2d252d84d28af1de93dc81e1a17ae2b55b60ec28ba8495ec2b666c37648998e0",
    "dynamics.py": "10382a89a86e813f0dd8526464f6256c3efdbed67556db38b20ae2a19f1391b3",
    "gain_schedule.py": "11595728f3caad390cd1f3844dc1003da4f0d63c89d77677831991467f844c84",
    "maneuver.py": "9e0e0c95660431db3dd7b14a3e83c60cedd6c984213204d88e1c30793ae9e806",
    "ndi.py": "ef19ce373cb5ae2dd0df9dedbfc40b990adaeba51227923a5e4d904b2fa94b0b",
    "ode45.py": "e50b29110f3c5aafd9060e785a150970664efd918ec5be167870cb05fc936026",
    "params.py": "7351aad00ad5212205855d84da395d096ec9261583b40a81b565df67d1c48d56",
    "quaternion.py": "6647b4c82d982991d0c182c181e13fe844d733e21d1be1e852d1daf12464b33e",
    "reference.py": "3fbcb27908870d8c9d1a61800857c9787bf157ee56127a5833626f3a727c92dc",
    "simulate.py": "34862186ad30231111253cdb58c4f7bf5e6c9f0b266ff65a9dd8a47fd84c12dc",
    "types.py": "8a6c22bb455310af680ed9e9ad93bda6d0438cb3e0e7d817800f869a1dd5b6dc",
}


def _after_docstring(text: str) -> bytes:
    """Everything after the module docstring's closing quotes."""
    parts = text.split('"""', 2)
    if len(parts) != 3:
        raise AssertionError("no module docstring found")
    return parts[2].encode()


class TestVendoredPortIsUnchanged(unittest.TestCase):
    def test_the_manifest_covers_exactly_the_vendored_files(self):
        on_disk = {path.name for path in _X31_DIR.glob("*.py")}
        self.assertEqual(on_disk, set(UPSTREAM_DIGESTS))

    def test_every_vendored_file_matches_its_upstream_body(self):
        for name, digest in sorted(UPSTREAM_DIGESTS.items()):
            with self.subTest(module=name):
                body = _after_docstring((_X31_DIR / name).read_text())
                self.assertEqual(hashlib.sha256(body).hexdigest(), digest)

    def test_every_vendored_file_keeps_the_gplv2_header_verbatim(self):
        for name in sorted(UPSTREAM_DIGESTS):
            with self.subTest(module=name):
                first = (_X31_DIR / name).read_text().split("\n", 1)[0]
                self.assertEqual(first, f'"""{_GPL_LINE}')

    def test_every_vendored_file_names_the_upstream_project_and_commit(self):
        for name in sorted(UPSTREAM_DIGESTS):
            with self.subTest(module=name):
                docstring = (_X31_DIR / name).read_text().split('"""', 2)[1]
                self.assertIn(_UPSTREAM, docstring)
                self.assertIn(_UPSTREAM_COMMIT, docstring)

    def test_the_package_docstring_still_opens_with_the_gpl_line(self):
        # The port's own `test_types.test_package_docstring` asserts this
        # docstring equals the GPL line alone. The provenance note the licence
        # handling requires makes that exact-equality test inapplicable here,
        # so it is superseded by this one: the GPL line must stay the first
        # line, and the note must follow it.
        lines = [line for line in (x31.__doc__ or "").splitlines() if line.strip()]
        self.assertEqual(lines[0], _GPL_LINE)
        self.assertIn(_UPSTREAM, x31.__doc__)


if __name__ == "__main__":
    unittest.main()
