"""A retiring supervisor must not stop or unlink a newer simulation session."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import sim_session


class SessionOwnershipTests(unittest.TestCase):
    def test_retiring_supervisor_does_not_touch_new_run(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'session.json'
            content=json.dumps(dict(run_dir='new',processes=[]))
            path.write_text(content)
            with patch.object(sim_session,'PIDFILE',path),patch.object(sim_session.os,'killpg') as signal:
                sim_session.stop(expected_run='old')
                signal.assert_not_called()
            self.assertEqual(path.read_text(),content)

    def test_concurrent_shutdown_cannot_unlink_replacement(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'session.json'
            path.write_text(json.dumps(dict(run_dir='old',processes=[dict(pid=999999999,start='unused')])))
            replacement=json.dumps(dict(run_dir='new',processes=[]))
            def changed_session(_):
                path.write_text(replacement)
                return None
            with patch.object(sim_session,'PIDFILE',path),patch.object(sim_session,'process_start',changed_session):
                sim_session.stop(expected_run='old')
            self.assertEqual(path.read_text(),replacement)


if __name__=='__main__':unittest.main()
