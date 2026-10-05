; NSIS installer for the directory staged by cmake/package_windows.ps1.
; The executable file stays simstudio.exe. The shortcut and the Programs entry show 应点仿真.
Unicode True
!define APP "应点仿真"
!define APP_EN "Ondot Sim"
!define PUBLISHER "应点 Ondot"
!ifndef STAGE
  !define STAGE "dist\OndotSim"
!endif
!ifndef OUTFILE
  !define OUTFILE "dist\OndotSim-setup.exe"
!endif

Name "${APP}"
OutFile "${OUTFILE}"
InstallDir "$PROGRAMFILES64\${APP_EN}"
RequestExecutionLevel admin

Page directory
Page instfiles
UninstPage uninstConfirm
UninstPage instfiles

Section "${APP}"
  SetOutPath "$INSTDIR"
  File /r "${STAGE}\*.*"
  WriteUninstaller "$INSTDIR\uninstall.exe"
  CreateShortcut "$SMPROGRAMS\${APP}.lnk" "$INSTDIR\simstudio.exe"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OndotSim" "DisplayName" "${APP}"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OndotSim" "UninstallString" "$INSTDIR\uninstall.exe"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OndotSim" "DisplayVersion" "0.7.0"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OndotSim" "Publisher" "${PUBLISHER}"
SectionEnd

Section "Uninstall"
  Delete "$SMPROGRAMS\${APP}.lnk"
  RMDir /r "$INSTDIR"
  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\OndotSim"
SectionEnd
