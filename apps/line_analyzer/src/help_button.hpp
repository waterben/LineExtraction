/// @file help_button.hpp
/// @brief Shared utility for adding a "?" help button to tool panels.

#pragma once

#include <QHBoxLayout>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <algorithm>

/// @brief Add a "?" help button at the top-right of a QMainWindow panel.
///
/// Inserts a right-aligned "?" QPushButton at the top of the central widget's
/// vertical layout.  Clicking it opens a QMessageBox with rich-text help.
/// The window's size constraints and geometry are adjusted so the extra row
/// does not clip existing content.
///
/// @param parent The QMainWindow to add the button to.
/// @param title  Window title for the help dialog.
/// @param html   Rich-text (HTML) help content.
/// @return Pointer to the created QPushButton.
inline QPushButton* addHelpButton(QMainWindow* parent, const QString& title, const QString& html) {
  auto* btn = new QPushButton("?", parent);
  btn->setFixedSize(24, 24);
  btn->setToolTip(QObject::tr("Show detailed help for this tool"));

  auto* row = new QHBoxLayout();
  row->setContentsMargins(0, 0, 0, 0);
  row->addStretch();
  row->addWidget(btn);

  if (auto* vbox = qobject_cast<QVBoxLayout*>(parent->centralWidget()->layout())) {
    vbox->insertLayout(0, row);
  }

  // Extra vertical space the button row occupies (button height + spacing).
  constexpr int kExtra = 30;

  const int maxH = parent->maximumHeight();
  const int minH = parent->minimumHeight();
  const int curH = parent->height();
  const int newH = curH + kExtra;

  // Relax maximum height first so that resize() and setMinimumHeight()
  // are not blocked by a too-tight upper bound.
  if (maxH < 16777215) {
    int newMax = maxH + kExtra;
    // Ensure max >= adjusted min and >= desired window height.
    if (minH > 0) {
      newMax = std::max(newMax, minH + kExtra);
    }
    newMax = std::max(newMax, newH);
    parent->setMaximumHeight(newMax);
  }

  // Grow the minimum height when one was explicitly set (> 0).
  if (minH > 0) {
    parent->setMinimumHeight(minH + kExtra);
  }

  // Actually resize the window so the added row is visible immediately.
  parent->resize(parent->width(), newH);

  QObject::connect(btn, &QPushButton::clicked, parent, [parent, title, html]() {
    QMessageBox box(parent);
    box.setWindowTitle(title);
    box.setTextFormat(Qt::RichText);
    box.setText(html);
    box.exec();
  });

  return btn;
}
