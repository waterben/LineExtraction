#include <qmainwindow.h>

#include <qplot3d/qwt3d_surfaceplot.h>
#include <qplot3d/qwt3d_function.h>
#include <qplot3d/qwt3d_plot.h>

#include "ui_axesmainwindowbase.h"

//MOC_SKIP_BEGIN
#if QT_VERSION < 0x040000
  class DummyBase : public AxesMainWindowBase
  {
  public:
    DummyBase(QWidget* parent = 0) 
      : AxesMainWindowBase(parent) 
    {
    } 
  };
#else
  class DummyBase : public QMainWindow, protected Ui::MainWindow
  {
  public:
    DummyBase(QWidget* parent = 0) 
      : QMainWindow(parent) 
    {
    } 
  };
#endif
//MOC_SKIP_END


class AxesMainWindow : public DummyBase
{
	Q_OBJECT

public:
	AxesMainWindow( QWidget* parent = 0);
	~AxesMainWindow();
	Qwt3D::SurfacePlot* plot;
	Qwt3D::Function *rosenbrock;
  void resetTics();

public slots:
	void setNumberGap(int gap);
	void setLabelGap(int gap);

  void setSmoothLines(bool);
  void setTicLength(int val);
  void setTicNumber(int degree);

  void standardItems();
  void complexItems();
  void letterItems();
  void timeItems();
  void customScale();

private:

  int tics;
};
