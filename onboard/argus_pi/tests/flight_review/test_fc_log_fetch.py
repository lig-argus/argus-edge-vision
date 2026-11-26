# mypy: ignore-errors
# 2026-11-26 ARGUS: 팀 mypy strict 적용은 후속 작업이다
"""No real device/network access: verify the file-only downloader with mocked FC."""

from pathlib import Path
from dataclasses import dataclass
import importlib.util, io, json, sys, tempfile, types, unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("fetch_fc_ulog", ROOT / "fetch_fc_ulog.py")
fetch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fetch)


@dataclass
class Row:
    name: str = "log101.ulg"
    is_dir: bool = False
    size_b: int = 16


class FetchTests(unittest.TestCase):
    def state(self, armed=False):
        return io.BytesIO(
            json.dumps(
                dict(
                    fresh=True, identity_ok=True, armed=armed, ground=not armed, expected_uid="test"
                )
            ).encode()
        )

    def test_armed_rejected_before_opening_diagnostic_endpoint(self):
        with (
            patch.object(sys, "argv", ["fetch", "--list"]),
            patch.object(fetch.urllib.request, "urlopen", return_value=self.state(True)),
            patch.object(fetch.socket, "socket") as sock,
        ):
            with self.assertRaises(RuntimeError):
                fetch.main()
            sock.assert_not_called()

    def test_only_mavftp_list_get_and_exact_bytes(self):
        link = Mock()
        link.message_hooks = []
        hb = Mock()
        hb.get_srcSystem.return_value = hb.get_srcComponent.return_value = 1
        link.recv_match.return_value = hb
        mavutil = types.SimpleNamespace(mavlink_connection=Mock(return_value=link))
        calls = []

        class FTP:
            def __init__(self, *args):
                self.list_result = [Row()]
                self.ftp_settings = types.SimpleNamespace()

            def cmd_list(self, args):
                calls.append(("list", args))
                return types.SimpleNamespace(error_code=0)

            def cmd_get(self, args):
                calls.append(("get", args[0]))
                Path(args[1]).write_bytes(b"x" * 16)
                return types.SimpleNamespace(error_code=0)

            def process_ftp_reply(self, *args, **kwargs):
                return types.SimpleNamespace(error_code=0)

        modules = {
            "pymavlink": types.SimpleNamespace(mavutil=mavutil),
            "pymavlink.mavftp": types.SimpleNamespace(MAVFTP=FTP),
        }
        with tempfile.TemporaryDirectory() as tmp:
            dest = Path(tmp) / "copy.ulg"
            with (
                patch.dict(sys.modules, modules),
                patch.object(
                    sys,
                    "argv",
                    [
                        "fetch",
                        "--remote",
                        "/fs/microsd/log/sess120/log101.ulg",
                        "--output",
                        str(dest),
                    ],
                ),
                patch.object(
                    fetch.urllib.request, "urlopen", side_effect=lambda *a, **k: self.state()
                ),
                patch.object(fetch.socket, "socket"),
                patch("builtins.print"),
            ):
                fetch.main()
            self.assertEqual(dest.read_bytes(), b"x" * 16)
            self.assertEqual([x[0] for x in calls], ["list", "get"])
            link.mav.assert_not_called()
            link.close.assert_called_once()
            self.assertEqual(mavutil.mavlink_connection.call_args.args[0], "udpin:127.0.0.1:14540")

    def test_path_outside_fc_logs_is_rejected(self):
        with (
            patch.object(
                sys, "argv", ["fetch", "--list", "--directory", "/fs/microsd/log/../secret"]
            ),
            patch.object(fetch.urllib.request, "urlopen") as net,
            patch.object(sys, "stderr", io.StringIO()),
        ):
            with self.assertRaises(SystemExit):
                fetch.main()
            net.assert_not_called()
