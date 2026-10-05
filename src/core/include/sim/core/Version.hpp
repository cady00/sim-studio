#pragma once

namespace sim {

// Visible English product name. The Chinese name is 应点仿真, in the 应点 Ondot series.
inline constexpr const char* kAppName = "Ondot Sim";
inline constexpr const char* kVersion = "0.7.0";
// QSettings and the crash-recovery directory keep this string so an existing
// session still restores. It is not the window title.
inline constexpr const char* kSettingsName = "Sim Studio";
inline constexpr int kProjectFormatVersion = 10;
inline constexpr const char* kProjectFormatId = "simstudio.project";

}  // namespace sim
