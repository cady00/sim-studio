; NSIS installer for the directory staged by cmake/package_windows.ps1.
!define APP "Sim Studio"
!ifndef STAGE
  !define STAGE "dist\SimStudio"
!endif
!ifndef OUTFILE
  !define OUTFILE "dist\SimStudio-0.1.0-win64-setup.exe"
!endif

Name "${APP}"
OutFile "${OUTFILE}"
Unicode True
InstallDir "$PROGRAMFILES64\Sim Studio"
RequestExecutionLevel admin

Page directory
Page instfiles
UninstPage uninstConfirm
UninstPage instfiles

Section "Sim Studio"
  SetOutPath "$INSTDIR"
  File /r "${STAGE}\*.*"
  WriteUninstaller "$INSTDIR\uninstall.exe"
  CreateShortcut "$SMPROGRAMS\Sim Studio.lnk" "$INSTDIR\simstudio.exe"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\SimStudio" "DisplayName" "${APP}"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\SimStudio" "UninstallString" "$INSTDIR\uninstall.exe"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\SimStudio" "DisplayVersion" "0.1.0"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\SimStudio" "Publisher" "Sim Studio"
SectionEnd

Section "Uninstall"
  Delete "$SMPROGRAMS\Sim Studio.lnk"
  RMDir /r "$INSTDIR"
  DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\SimStudio"
SectionEnd
