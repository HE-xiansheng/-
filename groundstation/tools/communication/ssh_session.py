import os
from pathlib import Path
import sys


def background_ssh():
    env = os.environ.copy()
    password_available = bool(env.get('GCS_SSH_PASSWORD'))
    options = ['-o', 'BatchMode=no' if password_available else 'BatchMode=yes']
    if sys.platform == 'linux':
        # The GUI establishes trust before starting these unattended sessions.
        options += ['-o', 'StrictHostKeyChecking=yes']
        if password_available:
            # Background workers can outlive the GUI's QTemporaryDir.
            env['SSH_ASKPASS'] = str(Path(__file__).with_name('gcs-askpass'))
            env['SSH_ASKPASS_REQUIRE'] = 'force'
    return options, env
