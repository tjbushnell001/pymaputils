#ifndef MAPS_SINGLE_MAP_LAYER_H_
#define MAPS_SINGLE_MAP_LAYER_H_

#include <ros/ros.h>
#include <memory>

#include <maps/map_layer.h>

#include "maps/utils/map_frame_utils.h"
#include "utils/file/file_utils.h"
#include "utils/map/utils.h"

namespace maps {

template <typename MapFeatureType>
class SingleMapLayer : public MapLayer
{
 public:
  /**
   * A map layer, which loads a single file into memory.
   *
   * This is templatized on MapFeatureType, which represents the concrete
   * feature type for each layer.
   *
   * This is an abstract class, which must be sub-classed.
   *
   * @param[in] layer_type The type of the layer.
   * @param[in] layer_name The name of the layer.  In instances where there's
   *            only one layer of a type, this should be the empty string.
   * @param[in] frame_type The target map frame.
   * @param[in] file_name The filename of the map file.
   **/
  SingleMapLayer(MapLayerType layer_type, const std::string& layer_name, MapFrameType frame_type,
                 const std::string& file_name)
    : MapLayer(layer_type, layer_name), loaded_(false), file_name_(file_name)
  {
    // set the target frame type
    map_frame_.type = frame_type;
  }

  /**
   * This will be called once when a layer is loaded.
   **/
  void initialLoad()
  {
    if (loaded_) {
      return;
    }

    // check that the file exists
    if (!file_utils::fileExists(file_name_)) {
      ROS_ERROR_STREAM(file_name_ << ": Path not found");
    } else if (!file_utils::isFile(file_name_)) {
      ROS_ERROR_STREAM(file_name_ << ": Not a file");
    }

    // if we're sticking with a global system that doesn't depend on location,
    // go ahead and load the map now
    // otherwise, it will get loaded with updateLocation()
    if (map_frame_.type == MapFrameType::GCS || map_frame_.type == MapFrameType::GCS_NED) {
      features_ = loadFeatures(file_name_, map_frame_);
      loaded_ = true;
    }
  }

  /**
   * Update the map layer to be centered around the given coordinates.
   *
   * @param[in] lat latitude
   * @param[in] lng longitude
   * @param[in] utm_zone UTM zone to use for utm transforms.
   **/
  void updateLocation(const double lat, const double lng, const map_utils::UtmZone utm_zone)
  {
    bool reload = !loaded_;

    // handle crossing UTM boundaries
    if (loaded_ && map_frame_.type == MapFrameType::UTM && map_frame_.utm_zone != utm_zone) {
      // utm zone has changed, reload file
      ROS_WARN_STREAM("Changed Utm zone from " << static_cast<int>(map_frame_.utm_zone.zone)
                                               << "to " << static_cast<int>(utm_zone.zone)
                                               << ", clearing tiles");
      reload = true;
    }

    // features will be the same map frame type, but center and utm zone can change
    map_frame_ = mapFrameFromLatLng(map_frame_.type, lat, lng, utm_zone);

    if (reload) {
      features_ = loadFeatures(file_name_, map_frame_);
      loaded_ = true;
    }
  }

  /**
   * Get the frame this layer is represented in.
   * @returns map frame
   **/
  MapFrame getMapFrame() const
  {
    return map_frame_;
  }

  /**
   * Get features for this layer.
   * @returns features
   **/
  std::shared_ptr<MapFeatureType> getFeatures()
  {
    return features_;
  }

 protected:
  /**
   * Load the specified file.
   *
   * This must be implemented by sub-classes.
   *
   * @param[in] file_name File to load.
   * @param[in] target_frame Map frame to convert tile into.
   **/
  virtual std::shared_ptr<MapFeatureType> loadFeatures(const std::string& filename,
                                                       const MapFrame& target_frame) = 0;

 private:
  std::string file_name_;
  bool loaded_ = false;
  std::shared_ptr<MapFeatureType> features_;
  MapFrame map_frame_;
};

}; // namespace maps

#endif // MAPS_SINGLE_MAP_LAYER_H_
