"""执行完整PowerShell入口，覆盖dot-source参数污染和日志启动顺序。"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1]
WSL = Path(os.environ.get('GCS_TEST_WSL_SOURCE', str(SOURCE)))
ROOT = SOURCE.parents[3]
PWSH = Path(os.environ.get('GCS_TEST_PWSH', str(ROOT / '.deps/powershell-test/ms-package/opt/microsoft/powershell/7/pwsh')))


class UninstallEntry(unittest.TestCase):
    def fixture(self, folder, broken=False):
        # 仅在临时测试副本替换平台/SID查询，所有卸载副作用均为写测试轨迹。
        text = (WSL / 'Uninstall-GroundStation.ps1').read_text(encoding='utf-8-sig')
        platform = '[Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or -not [Environment]::Is64BitProcess'
        identity = '[Security.Principal.WindowsIdentity]::GetCurrent().User.Value'
        self.assertEqual(text.count(platform), 1)
        self.assertEqual(text.count(identity), 1)
        text = text.replace(platform, '$false').replace(identity, "'gcs-entry-fixture'")
        script = folder / 'Uninstall-GroundStation.ps1'
        script.write_text(text, encoding='utf-8-sig')
        setup = '''param([switch]$FunctionsOnly)
function Invoke-GcsLinuxUninstall {param($StateDirectory) Add-Content -LiteralPath $env:GCS_ENTRY_TRACE -Value 'linux-uninstall'}
function Remove-GcsWindowsShortcuts {param($WslPath,$StateDirectory) Add-Content -LiteralPath $env:GCS_ENTRY_TRACE -Value 'shortcuts'}
function Invoke-HostPhase {param($Phase,$StateDirectory)
 if ($Phase -ne 'UninstallFirewall') {throw 'unexpected host phase'}
 Add-Content -LiteralPath $env:GCS_ENTRY_TRACE -Value 'firewall'
 return [pscustomobject]@{reason='fixture firewall done'}
}
function Save-SetupState {param($Directory,$Phase,$Message)
 if ($Phase -ne 'Uninstalled') {throw 'unexpected state'}
 Add-Content -LiteralPath $env:GCS_ENTRY_TRACE -Value 'complete'
}
if ($FunctionsOnly) {return}
throw 'Installer main must not run'
'''
        if broken:
            setup = "param([switch]$FunctionsOnly)\nthrow 'fixture import failure'\n"
        (folder / 'Setup-GroundStation.ps1').write_text(setup, encoding='utf-8-sig')
        return script

    def invoke(self, folder, script, *args):
        env = dict(os.environ, LOCALAPPDATA=str(folder / '用户 AppData'), SystemRoot=str(folder / 'Windows'), GCS_ENTRY_TRACE=str(folder / 'trace.txt'))
        return subprocess.run([str(PWSH), '-NoLogo', '-NoProfile', '-File', str(script), *args], env=env, capture_output=True, text=True, timeout=20)

    def test_normal_entry_runs_all_steps_despite_setup_functions_only_parameter(self):
        with tempfile.TemporaryDirectory(prefix='卸载入口 with spaces ') as temp:
            folder = Path(temp); script = self.fixture(folder)
            result = self.invoke(folder, script)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual((folder / 'trace.txt').read_text().splitlines(), ['linux-uninstall', 'shortcuts', 'firewall', 'complete'])
            logs = list((folder / '用户 AppData/ResearchFlightGCS-WSL-Setup').glob('uninstall-*.log'))
            self.assertEqual(len(logs), 1)
            self.assertIn('Uninstall entry started.', logs[0].read_text())
            self.assertIn('卸载完成', logs[0].read_text())

    def test_function_import_mode_still_has_no_uninstall_or_log_side_effect(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp); script = self.fixture(folder)
            result = self.invoke(folder, script, '-FunctionsOnly')
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertFalse((folder / 'trace.txt').exists())
            self.assertFalse((folder / '用户 AppData').exists())

    def test_dependency_failure_is_logged_before_import(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp); script = self.fixture(folder, broken=True)
            result = self.invoke(folder, script)
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            logs = list((folder / '用户 AppData/ResearchFlightGCS-WSL-Setup').glob('*.log'))
            self.assertEqual(len(logs), 1)
            self.assertIn('fixture import failure', logs[0].read_text())
            self.assertFalse((folder / 'trace.txt').exists())

    @unittest.skipIf(os.name == 'nt', '本项仅验证未改写的入口在Linux上报错并生成日志')
    def test_unmodified_real_entry_no_longer_silently_returns(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            result = self.invoke(folder, WSL / 'Uninstall-GroundStation.ps1')
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            logs = list((folder / '用户 AppData/ResearchFlightGCS-WSL-Setup').glob('*.log'))
            self.assertEqual(len(logs), 1)
            self.assertIn('64位Windows', logs[0].read_text())


if __name__ == '__main__':
    unittest.main()
