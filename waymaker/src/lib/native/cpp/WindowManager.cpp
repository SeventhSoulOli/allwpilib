// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "glass/WindowManager.h"

#include <algorithm>
#include <cstdio>

#include <fmt/format.h>
#include <wpigui.h>

using namespace glass;

WindowManager::WindowManager(std::string_view iniName)
    : m_iniSaver{iniName, this} {}

// read/write open state to ini file
void* WindowManager::IniSaver::IniReadOpen(const char* name) {
  return m_manager->GetOrAddWindow(name, true);
}

void WindowManager::IniSaver::IniReadLine(void* entry, const char* lineStr) {
  static_cast<Window*>(entry)->IniReadLine(lineStr);
}

void WindowManager::IniSaver::IniWriteAll(ImGuiTextBuffer* out_buf) {
  const char* typeName = GetTypeName();
  for (auto&& window : m_manager->m_windows) {
    window->IniWriteAll(typeName, out_buf);
  }
}

Window* WindowManager::AddWindow(std::string_view id,
                                 wpi::unique_function<void()> display) {
  auto win = GetOrAddWindow(id, false);
  if (!win) {
    return nullptr;
  }
  if (win->HasView()) {
    fmt::print(stderr, "GUI: ignoring duplicate window '{}'\n", id);
    return nullptr;
  }
  win->SetView(MakeFunctionView(std::move(display)));
  return win;
}

Window* WindowManager::AddWindow(std::string_view id,
                                 std::unique_ptr<View> view) {
  auto win = GetOrAddWindow(id, false);
  if (!win) {
    return nullptr;
  }
  if (win->HasView()) {
    fmt::print(stderr, "GUI: ignoring duplicate window '{}'\n", id);
    return nullptr;
  }
  win->SetView(std::move(view));
  return win;
}

Window* WindowManager::GetOrAddWindow(std::string_view id, bool duplicateOk) {
  // binary search
  auto it = std::lower_bound(
      m_windows.begin(), m_windows.end(), id,
      [](const auto& elem, std::string_view s) { return elem->GetId() < s; });
  if (it != m_windows.end() && (*it)->GetId() == id) {
    if (!duplicateOk) {
      fmt::print(stderr, "GUI: ignoring duplicate window '{}'\n", id);
      return nullptr;
    }
    return it->get();
  }
  // insert before (keeps sort)
  return m_windows.emplace(it, std::make_unique<Window>(id))->get();
}

Window* WindowManager::GetWindow(std::string_view id) {
  // binary search
  auto it = std::lower_bound(
      m_windows.begin(), m_windows.end(), id,
      [](const auto& elem, std::string_view s) { return elem->GetId() < s; });
  if (it == m_windows.end() || (*it)->GetId() != id) {
    return nullptr;
  }
  return it->get();
}

void WindowManager::GlobalInit() {
  wpi::gui::AddInit([this] { m_iniSaver.Initialize(); });
  wpi::gui::AddWindowScaler([this](float scale) {
    // scale default window positions
    for (auto&& window : m_windows) {
      window->ScaleDefault(scale);
    }
  });
  wpi::gui::AddLateExecute([this] { DisplayWindows(); });
}

void WindowManager::DisplayMenu() {
  for (auto&& window : m_windows) {
    window->DisplayMenuItem();
  }
}

void WindowManager::DisplayWindows() {
  for (auto&& window : m_windows) {
    window->Display();
  }
}
