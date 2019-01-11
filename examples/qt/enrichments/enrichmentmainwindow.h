#include <qplot3d/qwt3d_surfaceplot.h>
#include <qplot3d/qwt3d_function.h>
#include <qplot3d/qwt3d_plot.h>
#include "enrichments.h"

#include "ui_enrichmentmainwindowbase.h"


  class DummyBase : public QMainWindow, protected Ui::MainWindow
  {
  public:
    DummyBase(QWidget* parent = 0) 
      : QMainWindow(parent) 
    {
    } 
  };



class EnrichmentMainWindow : public DummyBase
{
	Q_OBJECT

public:
	EnrichmentMainWindow( QWidget* parent = 0 );
	~EnrichmentMainWindow();
  void setColor();
  Bar *bar;
  Qwt3D::SurfacePlot* plot;

public slots:
  void setLevel(int);
  void setWidth(int);
  void barSlot();
 
private:
  double level_, width_;
  
};


