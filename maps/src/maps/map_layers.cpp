#include <maps/map_layers.h>
#include <utils/ros/params.h>

using namespace maps;

MapLayers::MapLayers()
{
}

MapLayers::MapLayers(const std::vector<std::shared_ptr<MapLayer>>& layers)
{
  addLayers(layers);
}

void MapLayers::addLayer(std::shared_ptr<MapLayer> layer)
{
  layer->initialLoad();
  layers_[std::make_pair(layer->getLayerType(), layer->getLayerName())] = layer;
}


void MapLayers::addLayers(const std::vector<std::shared_ptr<MapLayer>>& layers)
{
  for (const auto& layer : layers) {
    addLayer(layer);
  }
}

bool MapLayers::hasLayer(MapLayerType layer_type, const std::string& layer_name) const
{
  return getLayer(layer_type, layer_name) != nullptr;
}

std::shared_ptr<MapLayer> MapLayers::getLayer(MapLayerType layer_type,
                                              const std::string& layer_name) const
{
  auto it = layers_.find(std::make_pair(layer_type, layer_name));
  if (it == layers_.end()) {
    return nullptr;
  }
  return it->second;
}

std::vector<std::pair<MapLayerType, std::string>> MapLayers::getLayerNames() const
{
  std::vector<std::pair<MapLayerType, std::string>> names;
  for (const auto& layer : layers_) {
    names.push_back(layer.first);
  }
  return names;
}

void MapLayers::updateLocation(const double lat, const double lng,
                               const map_utils::UtmZone utm_zone)
{
  for (const auto& layer : layers_) {
    layer.second->updateLocation(lat, lng, utm_zone);
  }
}

std::vector<std::shared_ptr<MapLayer>>
maps::createMapLayers(const std::vector<MapLayerType>& layer_types)
{
  std::vector<std::shared_ptr<MapLayer>> layers;

  for (const auto layer_type : layer_types) {
    switch (layer_type) {
      case MapLayerType::ROAD: {
        /**
         * Road Graph layers
         **/
	/*
        std::string map_dir;
        ros_params::getRequiredParam("/maps/map_dir", map_dir);

        const std::string road_tiles_dir = map_dir + "/road_tiles";
        auto layer = std::make_shared<RoadMapLayer>(road_tiles_dir, 1);
        layers.push_back(std::static_pointer_cast<MapLayer>(layer));
        */
      } break;

      case MapLayerType::LANE: {
        /**
         * Lane Map layers
         **/
	/*
        std::string map_dir;
        ros_params::getRequiredParam("/maps/map_dir", map_dir);

        const std::string lane_tiles_dir = map_dir + "/tiles";
        auto layer = std::make_shared<LaneMapLayer>(lane_tiles_dir, 1);
        layers.push_back(std::static_pointer_cast<MapLayer>(layer));
        */
      } break;

      case MapLayerType::LANE_ANNOTATION: {
        /**
         * Lane Annotation layers
         **/
	/*
        std::string map_dir;
        ros_params::getRequiredParam("/maps/map_dir", map_dir);

        const std::string lane_annotations_dir = map_dir + "/annotations";
        const std::vector<std::string> map_files =
            file_utils::getFilesInDirectory(lane_annotations_dir);
        for (const std::string& filename : map_files) {
          const std::string filename_stem = file_utils::getFilenameStem(filename);
          auto layer = std::make_shared<PolygonFeatureMapLayer>(
              MapLayerType::LANE_ANNOTATION, filename_stem, MapFrameType::GCS_NED, filename);
          layers.push_back(std::static_pointer_cast<MapLayer>(layer));
        }
        */
      } break;

      case MapLayerType::FREE_SPACE: {
        /**
         * Free Space layers
         **/
        std::string free_space_dir;
        ros_params::getRequiredParam("/maps/free_space_dir", free_space_dir);

        // create a layer for each map file
        for (const std::string& filename : file_utils::getFilesInDirectory(free_space_dir)) {
          const std::string filename_stem = file_utils::getFilenameStem(filename);
          if (filename_stem.empty())
            continue;
          auto layer = std::make_shared<FreeSpaceMapLayer>(filename_stem, filename);
          layers.push_back(std::static_pointer_cast<MapLayer>(layer));
        }
      } break;

      case MapLayerType::RADAR_ZONES: {
        /**
         * Radar Zones layers
         **/
        std::string radar_zones_dir;
        ros::NodeHandle nh;
        ros_params::getRequiredParam(&nh, "/maps/radar_zones_dir", radar_zones_dir);

        // create a layer for each map file
        for (const std::string& filename : file_utils::getFilesInDirectory(radar_zones_dir)) {
          const std::string filename_stem = file_utils::getFilenameStem(filename);
          auto layer =
              std::make_shared<GeoJsonMapLayer>(MapLayerType::RADAR_ZONES, filename_stem, filename);
          layers.push_back(std::static_pointer_cast<MapLayer>(layer));
        }
      } break;

      case MapLayerType::MAP_READER: {
        /**
         * Map Reader layers
         **/
        std::string map_reader_dir;
        ros_params::getRequiredParam("/maps/map_reader_dir", map_reader_dir);

        // create a layer for each map file
        for (const std::string& filename : file_utils::getFilesInDirectory(map_reader_dir)) {
          const std::string filename_stem = file_utils::getFilenameStem(filename);
          auto layer =
              std::make_shared<GeoJsonMapLayer>(MapLayerType::MAP_READER, filename_stem, filename);
          layers.push_back(std::static_pointer_cast<MapLayer>(layer));
        }
      } break;

      case MapLayerType::LOCALIZATION_ZONES: {
        /**
         * Localization Zones layers
         **/
	/*
        std::string map_dir;
        ros_params::getRequiredParam("/maps/map_dir", map_dir);

        const std::string filename = map_dir + "/../../localization_filter_zones.json";
        auto layer = std::make_shared<PolygonFeatureMapLayer>(MapLayerType::LOCALIZATION_ZONES, "",
                                                              MapFrameType::GCS_NED, filename);
        layers.push_back(std::static_pointer_cast<MapLayer>(layer));
        */
      } break;

      default:
        ROS_ERROR_STREAM("Map Layer " << static_cast<int>(layer_type) << " not supported");
    }
  }
  return layers;
}
