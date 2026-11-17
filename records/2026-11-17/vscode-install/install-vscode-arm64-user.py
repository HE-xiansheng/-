#!/usr/bin/env python3
"""Install the official ARM64 tarball for orangepi without administrator changes."""
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import uuid

home = Path.home()
assert home == Path('/home/orangepi') and platform.machine() == 'aarch64'
archive = Path(sys.argv[1]).resolve(strict=True)
base = home/'.local/opt/vscode'
binary = home/'.local/bin/code'
desktop = home/'.local/share/applications/code.desktop'
for existing in [base/'current', binary, desktop]:
    if existing.exists() or existing.is_symlink():
        raise RuntimeError('Refusing to replace an existing entry: '+str(existing))
base.mkdir(parents=True, exist_ok=True)
stage = Path(tempfile.mkdtemp(prefix='.install-', dir=base))
try:
    subprocess.run(['tar', '-xzf', str(archive), '--strip-components=1', '-C', str(stage)], check=True)
    version = json.loads((stage/'resources/app/package.json').read_text())['version']
    assert version and all(c.isalnum() or c in '.-+' for c in version)
    dependencies = subprocess.run(['ldd', str(stage/'code')], capture_output=True, text=True, check=True)
    if 'not found' in dependencies.stdout:
        raise RuntimeError('Missing VS Code libraries: '+dependencies.stdout)
    info = subprocess.run([str(stage/'bin/code'), '--version'], capture_output=True, text=True, check=True, timeout=20)
    target = base/version
    if target.exists():
        raise RuntimeError('Refusing to replace an existing release: '+str(target))
    stage.rename(target)
    temporary = base/('.current-'+uuid.uuid4().hex)
    temporary.symlink_to(target.name, target_is_directory=True)
    temporary.replace(base/'current')
    binary.parent.mkdir(parents=True, exist_ok=True)
    if binary.exists() or binary.is_symlink():
        raise RuntimeError('Refusing to replace an existing code command: '+str(binary))
    binary.symlink_to(base/'current/bin/code')
    desktop.parent.mkdir(parents=True, exist_ok=True)
    desktop.write_text('[Desktop Entry]\nVersion=1.0\nType=Application\nName=Visual Studio Code\n'
        'Comment=Code Editing\nExec='+str(base/'current/bin/code')+' --unity-launch %F\n'
        'Icon='+str(base/'current/resources/app/resources/linux/code.png')+'\n'
        'Terminal=false\nCategories=Development;IDE;TextEditor;\nStartupWMClass=Code\n')
    bashrc = home/'.bashrc'
    marker = '# VS Code user installation: command path'
    if marker not in bashrc.read_text():
        with bashrc.open('a') as shell:
            shell.write('\n'+marker+'\ncase ":$PATH:" in\n'
                        '  *":$HOME/.local/bin:"*) ;;\n'
                        '  *) export PATH="$HOME/.local/bin:$PATH" ;;\nesac\n')
    if shutil.which('update-desktop-database'):
        subprocess.run(['update-desktop-database', str(desktop.parent)], check=True)
    final = subprocess.run([str(binary), '--version'], capture_output=True, text=True, check=True, timeout=20)
    print(json.dumps({'version': version, 'path': str(target), 'command': str(binary),
                     'desktop': str(desktop), 'version_output': final.stdout,
                     'interactive_shell_path_configured': True,
                     'dependencies_resolved': True, 'installation': 'per-user official ARM64 archive'}, indent=2))
finally:
    if stage.exists():
        shutil.rmtree(stage)
