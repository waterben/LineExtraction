/// @file help_viewer.h
/// @brief Modeless Markdown help viewer for the Line Analyzer.
///
/// Shows project READMEs in a QTextBrowser with navigation and TOC.
/// The viewer is a singleton dialog — all help buttons share the same window.

#pragma once

#include <QComboBox>
#include <QDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QSplitter>
#include <QTextBrowser>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

/// @brief Modeless help viewer that renders Markdown README files.
///
/// Usage:
/// @code
///   HelpViewer::show("extensions/accuracy/README.md", "settings");
/// @endcode
///
/// The viewer resolves README paths relative to the app source tree.
/// Clicking internal `[link](../foo/README.md)` links navigates within
/// the viewer.  External links are opened in the system browser.
class HelpViewer : public QDialog {
  Q_OBJECT
  Q_DISABLE_COPY(HelpViewer)

 public:
  /// @brief Show the help viewer at a specific README and optional anchor.
  ///
  /// If the viewer is already open, it navigates to the requested page.
  /// The @p readme_relative path is relative to `apps/line_analyzer/`.
  ///
  /// @param readme_relative  Relative path, e.g. "extensions/accuracy/README.md".
  /// @param anchor           Optional HTML anchor id (heading slug) to scroll to.
  /// @param parent           Parent widget (used only for initial creation).
  static void showHelp(const QString& readme_relative, const QString& anchor = {}, QWidget* parent = nullptr);

  /// @brief Get (or create) the singleton instance.
  static HelpViewer* instance(QWidget* parent = nullptr);

 protected:
  /// @brief Handle link clicks — navigate internal .md links, open external.
  void handleAnchorClicked(const QUrl& url);

 private slots:
  /// @brief Navigate back in history.
  void goBack();

  /// @brief Navigate forward in history.
  void goForward();

  /// @brief TOC tree item selected — scroll to that heading.
  void tocItemActivated(QTreeWidgetItem* item, int column);

 private:
  /// @brief Private constructor — use showHelp() or instance().
  explicit HelpViewer(QWidget* parent = nullptr);

  /// @brief Handle mouse back/forward buttons.
  void mousePressEvent(QMouseEvent* event) override;

  /// @brief Event filter for mouse navigation on child widgets.
  bool eventFilter(QObject* obj, QEvent* event) override;

  /// @brief Resolve the base directory containing the READMEs.
  ///
  /// Searches BUILD_WORKSPACE_DIRECTORY, relative paths, and runfiles
  /// for the `apps/line_analyzer/` source tree.
  void resolveBasePath();

  /// @brief Load and render a README file.
  ///
  /// @param relative  Path relative to `apps/line_analyzer/`, e.g. "README.md".
  /// @param anchor    Optional heading anchor to scroll to.
  void loadReadme(const QString& relative, const QString& anchor = {});

  /// @brief Build the TOC tree from the currently loaded Markdown.
  void buildTocTree(const std::string& markdown);

  /// @brief Update back/forward button enabled state.
  void updateNavButtons();

  // --- Widgets ---
  QTextBrowser* browser_{nullptr};
  QTreeWidget* toc_tree_{nullptr};
  QPushButton* btn_back_{nullptr};
  QPushButton* btn_forward_{nullptr};
  QLabel* lbl_title_{nullptr};

  // --- State ---
  std::filesystem::path base_path_;  ///< Resolved path to `apps/line_analyzer/`.
  QString current_relative_;         ///< Currently displayed README path.
  std::string current_markdown_;     ///< Raw Markdown of current file.

  // --- Navigation history ---
  struct HistoryEntry {
    QString relative{};
    QString anchor{};
  };
  std::vector<HistoryEntry> history_;
  int history_pos_{-1};
  bool navigating_{false};  ///< Suppress history push during back/forward.

  // --- Cache ---
  std::unordered_map<std::string, std::string> cache_;  ///< relative → raw Markdown.

  static HelpViewer* instance_;
};
