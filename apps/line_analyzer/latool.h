#ifndef LA_TOOL_H
#define LA_TOOL_H

#include <QtWidgets/QMainWindow>
#include <vector>

class ControlWindow;

class LATool : public QMainWindow {
    QString name_;

public:
    LATool(const QString n, QWidget * parent = 0) : name_(n), QMainWindow(parent) {}
    
    virtual void connectTools(ControlWindow* la) = 0;
    inline const QString name() const { return name_; }
};

typedef std::vector<LATool*> LATools;

#endif // LA_TOOL_H
