#include "time.hh"

namespace dynamicgraph {
namespace agimus {

template <>
std::string Time<int>::typename_ = "int";

template <>
std::string Time<int64_t>::typename_ = "int64";

typedef Time<sigtime_t> _Time;
template<>
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (_Time, "Time");

} // namespace agimus
} // namespace dynamicgraph
