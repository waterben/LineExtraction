/// @file help_viewer.cpp
/// @brief Implementation of the Markdown help viewer.

#include "help_viewer.h"

#include "markdown_converter.hpp"

#include <QDesktopServices>
#include <QFont>
#include <QHeaderView>
#include <QIcon>
#include <QMouseEvent>
#include <QScrollBar>
#include <QShortcut>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

HelpViewer* HelpViewer::instance_ = nullptr;

HelpViewer* HelpViewer::instance(QWidget* parent) {
  if (!instance_) {
    instance_ = new HelpViewer(parent);
  }
  return instance_;
}

void HelpViewer::showHelp(const QString& readme_relative, const QString& anchor, QWidget* parent) {
  auto* viewer = instance(parent);
  viewer->loadReadme(readme_relative, anchor);
  viewer->show();
  viewer->raise();
  viewer->activateWindow();
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

HelpViewer::HelpViewer(QWidget* parent)
    : QDialog(parent), base_path_(), current_relative_(), current_markdown_(), history_(), cache_() {
  setWindowTitle(tr("Line Analyzer — Help"));
  setWindowFlags(windowFlags() | Qt::WindowMinMaxButtonsHint);
  resize(860, 640);

  // --- Navigation toolbar ---
  auto* nav_bar = new QHBoxLayout();
  nav_bar->setContentsMargins(4, 4, 4, 0);

  btn_back_ = new QPushButton(tr("◀ Back"), this);
  btn_back_->setFixedWidth(80);
  btn_back_->setEnabled(false);
  btn_back_->setToolTip(tr("Go to previous page"));

  btn_forward_ = new QPushButton(tr("Forward ▶"), this);
  btn_forward_->setFixedWidth(80);
  btn_forward_->setEnabled(false);
  btn_forward_->setToolTip(tr("Go to next page"));

  lbl_title_ = new QLabel(this);
  lbl_title_->setAlignment(Qt::AlignCenter);
  QFont title_font = lbl_title_->font();
  title_font.setBold(true);
  lbl_title_->setFont(title_font);

  nav_bar->addWidget(btn_back_);
  nav_bar->addWidget(btn_forward_);
  nav_bar->addSpacing(12);
  nav_bar->addWidget(lbl_title_, 1);

  // --- TOC tree ---
  toc_tree_ = new QTreeWidget(this);
  toc_tree_->setHeaderHidden(true);
  toc_tree_->setMaximumWidth(240);
  toc_tree_->setMinimumWidth(160);
  toc_tree_->setToolTip(tr("Table of contents — click to jump to a section"));

  // --- Browser ---
  browser_ = new QTextBrowser(this);
  browser_->setOpenLinks(false);  // We handle link clicks ourselves.
  browser_->setOpenExternalLinks(false);

  // --- Splitter (TOC | Browser) ---
  auto* splitter = new QSplitter(Qt::Horizontal, this);
  splitter->addWidget(toc_tree_);
  splitter->addWidget(browser_);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({200, 660});

  // --- Main layout ---
  auto* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->addLayout(nav_bar);
  main_layout->addWidget(splitter, 1);

  // --- Connections ---
  connect(btn_back_, &QPushButton::clicked, this, &HelpViewer::goBack);
  connect(btn_forward_, &QPushButton::clicked, this, &HelpViewer::goForward);
  connect(browser_, &QTextBrowser::anchorClicked, this, &HelpViewer::handleAnchorClicked);
  connect(toc_tree_, &QTreeWidget::itemActivated, this, &HelpViewer::tocItemActivated);
  connect(toc_tree_, &QTreeWidget::itemClicked, this, &HelpViewer::tocItemActivated);

  // Keyboard shortcuts.
  auto* esc = new QShortcut(QKeySequence(Qt::Key_Escape), this);
  connect(esc, &QShortcut::activated, this, &QDialog::close);

  auto* shortcut_back = new QShortcut(QKeySequence(Qt::ALT | Qt::Key_Left), this);
  connect(shortcut_back, &QShortcut::activated, this, &HelpViewer::goBack);
  auto* shortcut_fwd = new QShortcut(QKeySequence(Qt::ALT | Qt::Key_Right), this);
  connect(shortcut_fwd, &QShortcut::activated, this, &HelpViewer::goForward);

  auto* shortcut_backspace = new QShortcut(QKeySequence(Qt::Key_Backspace), this);
  connect(shortcut_backspace, &QShortcut::activated, this, &HelpViewer::goBack);

  // Install event filter on browser for mouse back/forward buttons.
  browser_->viewport()->installEventFilter(this);
  browser_->installEventFilter(this);

  // Resolve the base path for README files.
  resolveBasePath();
}

// ---------------------------------------------------------------------------
// Path resolution
// ---------------------------------------------------------------------------

void HelpViewer::resolveBasePath() {
  namespace fs = std::filesystem;

  // Helper: check that a candidate directory contains the main README.
  auto isValid = [](const fs::path& candidate) { return fs::exists(candidate / "README.md"); };

  // 1. BUILD_WORKSPACE_DIRECTORY (set by `bazel run`).
  if (const char* ws = std::getenv("BUILD_WORKSPACE_DIRECTORY")) {
    fs::path candidate = fs::path(ws) / "apps" / "line_analyzer";
    if (isValid(candidate)) {
      base_path_ = candidate;
      return;
    }
  }

  // 2. Relative paths from cwd.
  for (const auto& rel : {"apps/line_analyzer", "../apps/line_analyzer", "../../apps/line_analyzer"}) {
    fs::path candidate = fs::absolute(rel);
    if (isValid(candidate)) {
      base_path_ = candidate;
      return;
    }
  }

  // 3. RUNFILES_DIR (Bazel hermetic mode).
  if (const char* rd = std::getenv("RUNFILES_DIR")) {
    for (const auto& repo : {"_main", "line_extraction"}) {
      fs::path candidate = fs::path(rd) / repo / "apps" / "line_analyzer";
      if (isValid(candidate)) {
        base_path_ = candidate;
        return;
      }
    }
  }

  std::cerr << "HelpViewer: could not resolve apps/line_analyzer/ base path" << std::endl;
}

// ---------------------------------------------------------------------------
// Loading & rendering
// ---------------------------------------------------------------------------

void HelpViewer::loadReadme(const QString& relative, const QString& anchor) {
  namespace fs = std::filesystem;

  if (base_path_.empty()) {
    browser_->setHtml(
        "<html><body><h2>Help not available</h2>"
        "<p>Could not find the documentation files. "
        "Make sure you are running from the workspace root or via "
        "<code>bazel run</code>.</p></body></html>");
    return;
  }

  // Read the file (with caching).
  std::string rel_std = relative.toStdString();
  std::string markdown;
  auto it = cache_.find(rel_std);
  if (it != cache_.end()) {
    markdown = it->second;
  } else {
    fs::path full_path = base_path_ / rel_std;
    if (!fs::exists(full_path)) {
      browser_->setHtml(
          "<html><body><h2>File not found</h2>"
          "<p>Could not find: <code>" +
          QString::fromStdString(full_path.string()) + "</code></p></body></html>");
      return;
    }
    std::ifstream ifs(full_path);
    if (!ifs) {
      browser_->setHtml("<html><body><h2>Read error</h2></body></html>");
      return;
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    markdown = ss.str();
    cache_[rel_std] = markdown;
  }

  // Convert and display.
  std::string html = lsfm::markdownToHtml(markdown);
  current_relative_ = relative;
  current_markdown_ = markdown;

  // Set the search paths so QTextBrowser can find relative images.
  fs::path readme_dir = (base_path_ / rel_std).parent_path();
  browser_->setSearchPaths({QString::fromStdString(readme_dir.string())});

  browser_->setHtml(QString::fromStdString(html));

  // Scroll to anchor if provided.
  if (!anchor.isEmpty()) {
    browser_->scrollToAnchor(anchor);
  }

  // Build TOC.
  buildTocTree(markdown);

  // Update title.
  // Extract the first H1 heading as display title.
  auto toc = lsfm::extractToc(markdown);
  QString title;
  for (const auto& entry : toc) {
    if (entry.level == 1) {
      title = QString::fromStdString(entry.text);
      break;
    }
  }
  if (title.isEmpty()) title = relative;
  lbl_title_->setText(title);

  // Push to history.
  if (!navigating_) {
    // Truncate forward history.
    if (history_pos_ + 1 < static_cast<int>(history_.size())) {
      history_.resize(static_cast<size_t>(history_pos_ + 1));
    }
    history_.push_back({relative, anchor});
    history_pos_ = static_cast<int>(history_.size()) - 1;
  }
  updateNavButtons();
}

void HelpViewer::buildTocTree(const std::string& markdown) {
  toc_tree_->clear();
  auto toc = lsfm::extractToc(markdown);

  // Map H1 → top-level, H2 → child of last H1, H3 → child of last H2, etc.
  std::vector<QTreeWidgetItem*> stack;  // Current parent at each level.

  for (const auto& entry : toc) {
    auto* item = new QTreeWidgetItem();
    item->setText(0, QString::fromStdString(entry.text));
    item->setData(0, Qt::UserRole, QString::fromStdString(entry.anchor));

    // Find appropriate parent.
    while (static_cast<int>(stack.size()) >= entry.level) {
      stack.pop_back();
    }

    if (stack.empty()) {
      toc_tree_->addTopLevelItem(item);
    } else {
      stack.back()->addChild(item);
    }
    stack.push_back(item);
  }

  toc_tree_->expandAll();
}

void HelpViewer::updateNavButtons() {
  btn_back_->setEnabled(history_pos_ > 0);
  btn_forward_->setEnabled(history_pos_ + 1 < static_cast<int>(history_.size()));
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void HelpViewer::handleAnchorClicked(const QUrl& url) {
  QString scheme = url.scheme();
  QString path = url.path();

  // External link — open in system browser.
  if (scheme == "http" || scheme == "https") {
    QDesktopServices::openUrl(url);
    return;
  }

  // Fragment-only link (same page).
  if (path.isEmpty() && url.hasFragment()) {
    browser_->scrollToAnchor(url.fragment());
    return;
  }

  // Internal .md link — resolve relative to current README.
  if (path.endsWith(".md", Qt::CaseInsensitive)) {
    namespace fs = std::filesystem;
    // Resolve relative to current README's directory.
    fs::path current_dir = fs::path(current_relative_.toStdString()).parent_path();
    fs::path target = (current_dir / path.toStdString()).lexically_normal();
    QString new_relative = QString::fromStdString(target.string());

    // Check if the resolved file actually exists before navigating.
    fs::path full_path = base_path_ / target;
    if (!fs::exists(full_path)) {
      // Show a non-intrusive inline message instead of navigating.
      browser_->setHtml(
          "<html><body>"
          "<h2>Document not available</h2>"
          "<p>The linked document <code>" +
          new_relative +
          "</code> is not available in the current deployment.</p>"
          "<p>This documentation is part of the project source tree. To view it, "
          "open the file directly in your editor or run the Line Analyzer from "
          "the workspace root with <code>bazel run</code>.</p>"
          "<p><a href=\"javascript:void(0)\" "
          "style=\"color:#0366d6\">Use the Back button to return.</a></p>"
          "</body></html>");
      return;
    }

    loadReadme(new_relative, url.fragment());
    return;
  }

  // Non-.md file links (source files, etc.) — ignore silently.
  // These are typically references to .cpp/.hpp files that can't be rendered.
}

void HelpViewer::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::BackButton) {
    goBack();
    event->accept();
    return;
  }
  if (event->button() == Qt::ForwardButton) {
    goForward();
    event->accept();
    return;
  }
  QDialog::mousePressEvent(event);
}

bool HelpViewer::eventFilter(QObject* obj, QEvent* event) {
  if (event->type() == QEvent::MouseButtonPress) {
    auto* me = static_cast<QMouseEvent*>(event);
    if (me->button() == Qt::BackButton) {
      goBack();
      return true;
    }
    if (me->button() == Qt::ForwardButton) {
      goForward();
      return true;
    }
  }
  return QDialog::eventFilter(obj, event);
}

void HelpViewer::goBack() {
  if (history_pos_ <= 0) return;
  --history_pos_;
  navigating_ = true;
  const auto& entry = history_[static_cast<size_t>(history_pos_)];
  loadReadme(entry.relative, entry.anchor);
  navigating_ = false;
  updateNavButtons();
}

void HelpViewer::goForward() {
  if (history_pos_ + 1 >= static_cast<int>(history_.size())) return;
  ++history_pos_;
  navigating_ = true;
  const auto& entry = history_[static_cast<size_t>(history_pos_)];
  loadReadme(entry.relative, entry.anchor);
  navigating_ = false;
  updateNavButtons();
}

void HelpViewer::tocItemActivated(QTreeWidgetItem* item, int /*column*/) {
  if (!item) return;
  QString anchor = item->data(0, Qt::UserRole).toString();
  if (!anchor.isEmpty()) {
    browser_->scrollToAnchor(anchor);
  }
}
