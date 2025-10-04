#pragma once

#include "qwt3d_global.h"
#include "qwt3d_types.h"

#include <qstring.h>

namespace Qwt3D {

//! Abstract base class for general mappings
/**

*/
class QWT3D_EXPORT Mapping {
 public:
  virtual ~Mapping() {}                                 //!< Destructor.
  virtual QString name() const { return QString(""); }  //!< Descriptive String.
};


}  // namespace Qwt3D
