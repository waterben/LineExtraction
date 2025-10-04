#pragma once

#include <qplot3d/qwt3d_plot.h>


class Bar : public Qwt3D::VertexEnrichment {
 public:
  Bar();
  Bar(double rad, double level);

  Qwt3D::Enrichment* clone() const { return new Bar(*this); }

  void configure(double rad, double level);
  void drawBegin();
  void drawEnd();
  void draw(Qwt3D::Triple const&);

 private:
  double level_, radius_;
  double diag_;
};

class Label3D {
 public:
  void draw(Qwt3D::Triple const&, double w, double h);
};
