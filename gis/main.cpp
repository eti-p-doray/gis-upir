#include <stdio.h>

#include <geometry.hpp>
#include <geometry/geometries/point.hpp>
#include <geometry/geometries/point_xy.hpp>
#include <geometry/geometries/register/point.hpp>
#include <geometry/geometries/multi_polygon.hpp>
#include <geometry/extensions/gis/latlong/latlong.hpp>
#include <geometry/extensions/gis/projections/project_transformer.hpp>
#include <geometry/extensions/gis/projections/project_inverse_transformer.hpp>
#include <geometry/extensions/gis/projections/epsg.hpp>

#include "shapefile.hpp"
//#include "mapped_file.hpp"
//#include "csv.hpp"
//#include "shapefile.hpp"
//#include "tuple.hpp"

//using namespace boost::geometry;

namespace gis {

namespace bg = boost::geometry;

template <class From, class To, class Strategy = void>
struct transform {
  Strategy strategy;
  transform(const Strategy& s) : strategy(s) {}

  To operator()(const From& x) const {
    To y;
    bg::transform(x, y, strategy);
    return y;
  }
};

template <class From, class To>
struct transform<From, To, void> {
  To operator()(const From& x) const {
    To y;
    bg::transform(x, y);
    return y;
  }
};

template <class From, class To>
using project = transform<From, To, bg::projections::project_transformer<From, To>>;
template <class From, class To>
using unproject = transform<From, To, bg::projections::project_inverse_transformer<From, To>>;

using shp = bg::model::point<double, 2, bg::cs::cartesian>;
template <class Geometry>
using shp_transform = 
  transform<shp, typename bg::point_type<Geometry>::type>;

template <class SrcGeometry, class DstGeometry, class TransformFcn = shp_transform<DstGeometry>>
struct convert {};

template <class DstGeometry, class TransformFcn>
struct convert<io::shp::point, DstGeometry, TransformFcn> {
  TransformFcn fn;
  convert(const TransformFcn& f) : fn(f) {}

  DstGeometry operator()(const io::shp::point& g) const {
    return fn(shp(g.x, g.y));
  }
};

template <class DstGeometry, class TransformFcn>
struct convert<io::shp::multipoint, DstGeometry, TransformFcn> {
  using point = typename bg::point_type<DstGeometry>::type;

  convert<io::shp::point, point, TransformFcn> convert_point;
  convert(const TransformFcn& f) : convert_point(f) {}

  DstGeometry operator()(const io::shp::multipoint& g) const {
    DstGeometry dst;
    for (size_t i = 0; i < g.num_points; ++i) {
      bg::append(dst, convert_point(g.points[i]));
    }
    return dst;
  }
};

}

namespace bg = boost::geometry;
namespace shp = io::shp;
using point_ll = bg::model::ll::point<bg::degree>;
using unproject = gis::unproject<gis::shp, point_ll>;

int main(int argc, char **argv) {
  io::shapefile_source in(
    "intersections/intersections_on_bike_network_with_change_in_road_type");
  auto in_view = in();

  gis::convert<shp::point, point_ll, unproject> point({bg::projections::init(32188)});
  gis::convert<shp::multipoint, point_ll, unproject> multipoint({bg::projections::init(32188)});

  for (auto&& record : in_view) {
    std::cout << bg::wkt(point(record.geometry)) << std::endl;
  }
}
