/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file polar_to_cart_converter.h
 *
 *  \brief Converts point clouds from polar coordinates to cartesian
 *  coordinates, i.e. HVDIR to XYZIR.
 */

#ifndef QUANERGY_MODULES_POLAR_TO_CART_CONVERTER_H
#define QUANERGY_MODULES_POLAR_TO_CART_CONVERTER_H

#include <memory>

#include <boost/signals2.hpp>

#include <pcl/point_cloud.h>

#include <quanergy/common/point_hvdir.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT PolarToCartConverter
    {
      typedef std::shared_ptr<PolarToCartConverter> Ptr;

      typedef PointCloudXYZIRPtr ResultType;

      typedef boost::signals2::signal<void (const ResultType&)> Signal;

      boost::signals2::connection connect(const typename Signal::slot_type& subscriber);

      void slot(PointCloudHVDIRConstPtr const &);

    private:

      static PointCloudXYZIR::PointType polarToCart(PointCloudHVDIR::PointType const & from);

      Signal signal_;
    };

  } // namespace client

} // namespace quanergy


#endif
