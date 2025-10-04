#pragma once

#include "qwt3d_mapping.h"

namespace Qwt3D {

class SurfacePlot;


//! Abstract base class for mappings acting on rectangular grids
/**

*/
class QWT3D_EXPORT GridMapping : public Mapping {
 public:
  GridMapping();  //!< Constructs GridMapping object w/o assigned SurfacePlot.
  Q_DISABLE_COPY(GridMapping)

  void setMesh(unsigned int columns, unsigned int rows);               //!< Sets number of rows and columns.
  void setDomain(double minu, double maxu, double minv, double maxv);  //!< Sets u-v domain boundaries.
  void restrictRange(Qwt3D::ParallelEpiped const&);  //!< Restrict the mappings range to the parallelepiped

 protected:
  Qwt3D::ParallelEpiped range_p;
  Qwt3D::SurfacePlot* plotwidget_p{nullptr};
  unsigned int umesh_p{};
  unsigned int vmesh_p{};
  double minu_p{};
  double maxu_p{};
  double minv_p{};
  double maxv_p{};
};

}  // namespace Qwt3D
