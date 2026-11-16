Unicode true
Target amd64-unicode
!include "MUI2.nsh"
!include "x64.nsh"
Name "科研飞行平台-地面站"
OutFile "${OUTPUT}"
InstallDir "$LOCALAPPDATA\Programs\ResearchFlightGroundStation"
InstallDirRegKey HKCU "Software\ResearchFlightGroundStation" "InstallDir"
RequestExecutionLevel user
SetCompressor /SOLID lzma
SetCompressorDictSize 32
ShowInstDetails show
ShowUninstDetails show
!define MUI_ICON "${BUNDLE}/resources/groundstation.ico"
!define MUI_UNICON "${BUNDLE}/resources/groundstation.ico"
!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "SimpChinese"
VIProductVersion "1.0.26.1109"
VIAddVersionKey /LANG=2052 "ProductName" "科研飞行平台-地面站"
VIAddVersionKey /LANG=2052 "ProductVersion" "${VERSION}"
VIAddVersionKey /LANG=2052 "FileDescription" "科研飞行平台-地面站安装程序"
VIAddVersionKey /LANG=2052 "FileVersion" "${VERSION}"
VIAddVersionKey /LANG=2052 "LegalCopyright" "科研飞行平台"
!macro RequireStopped suffix
  IfFileExists "$INSTDIR\bin\customgeometry.exe" 0 done_check_${suffix}
    System::Call 'kernel32::CreateFileW(w "$INSTDIR\bin\customgeometry.exe", i 0x40000000, i 0, p 0, i 3, i 0x80, p 0) p .r0'
    ${If} $0 == -1
      MessageBox MB_OK|MB_ICONSTOP "请先关闭地面站，再进行安装或卸载。"
      Abort
    ${EndIf}
    System::Call 'kernel32::CloseHandle(p r0)'
  done_check_${suffix}:
!macroend
Function .onInit
  ${IfNot} ${RunningX64}
    MessageBox MB_OK|MB_ICONSTOP "此版本需要64位Windows。"
    Abort
  ${EndIf}
FunctionEnd
Function .onVerifyInstDir
  !insertmacro RequireStopped install
FunctionEnd
Function un.onInit
  !insertmacro RequireStopped uninstall
FunctionEnd
Section "地面站" SEC_MAIN
  !insertmacro RequireStopped section
  SetShellVarContext current
  SetOutPath "$INSTDIR"
  File /r "${BUNDLE}/*"
  WriteUninstaller "$INSTDIR\Uninstall.exe"
  CreateDirectory "$SMPROGRAMS\科研飞行平台"
  CreateShortcut "$SMPROGRAMS\科研飞行平台\地面站.lnk" "$INSTDIR\bin\customgeometry.exe" "--cloud-port 9000 --pose-port 9001 --preflight-port 9003" "$INSTDIR\resources\groundstation.ico"
  CreateShortcut "$DESKTOP\科研飞行平台-地面站.lnk" "$INSTDIR\bin\customgeometry.exe" "--cloud-port 9000 --pose-port 9001 --preflight-port 9003" "$INSTDIR\resources\groundstation.ico"
  WriteRegStr HKCU "Software\ResearchFlightGroundStation" "InstallDir" "$INSTDIR"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\ResearchFlightGroundStation" "DisplayName" "科研飞行平台-地面站"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\ResearchFlightGroundStation" "DisplayVersion" "${VERSION}"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\ResearchFlightGroundStation" "UninstallString" '$\"$INSTDIR\Uninstall.exe$\"'
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\ResearchFlightGroundStation" "DisplayIcon" "$INSTDIR\resources\groundstation.ico"
  WriteRegDWORD HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\ResearchFlightGroundStation" "NoModify" 1
  WriteRegDWORD HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\ResearchFlightGroundStation" "NoRepair" 1
SectionEnd
Section "Uninstall"
  SetShellVarContext current
  IfFileExists "$INSTDIR\bin\logs\*.*" 0 +3
    CreateDirectory "$LOCALAPPDATA\ResearchFlightGroundStation\uninstall-logs"
    CopyFiles /SILENT "$INSTDIR\bin\logs\*.*" "$LOCALAPPDATA\ResearchFlightGroundStation\uninstall-logs"
  Delete "$DESKTOP\科研飞行平台-地面站.lnk"
  Delete "$SMPROGRAMS\科研飞行平台\地面站.lnk"
  RMDir "$SMPROGRAMS\科研飞行平台"
  RMDir /r "$INSTDIR\bin"
  RMDir /r "$INSTDIR\runtime"
  RMDir /r "$INSTDIR\tools"
  RMDir /r "$INSTDIR\licenses"
  RMDir /r "$INSTDIR\resources"
  Delete "$INSTDIR\使用说明.txt"
  Delete "$INSTDIR\THIRD_PARTY_NOTICES.txt"
  Delete "$INSTDIR\payload-manifest.json"
  Delete "$INSTDIR\Uninstall.exe"
  RMDir "$INSTDIR"
  DeleteRegKey HKCU "Software\ResearchFlightGroundStation"
  DeleteRegKey HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\ResearchFlightGroundStation"
SectionEnd
