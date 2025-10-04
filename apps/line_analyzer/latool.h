#pragma once

#include <QtWidgets/QMainWindow>

#include <vector>

class ControlWindow;

class LATool : public QMainWindow {
  QString name_;

 public:
  LATool(const QString& n, QWidget* parent = 0) : QMainWindow(parent), name_(n) {}

  virtual void connectTools(ControlWindow* la) = 0;
  inline const QString name() const { return name_; }
};

typedef std::vector<LATool*> LATools;
