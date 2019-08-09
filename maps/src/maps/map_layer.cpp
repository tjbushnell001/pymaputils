#include <maps/map_layer.h>

using namespace maps;

MapLayer::MapLayer(MapLayerType layer_type, const std::string& layer_name)
  : layer_type_(layer_type), layer_name_(layer_name)
{
}

MapLayerType MapLayer::getLayerType() const
{
  return layer_type_;
}

const std::string& MapLayer::getLayerName() const
{
  return layer_name_;
}
