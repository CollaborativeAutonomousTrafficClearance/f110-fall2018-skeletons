#ifndef COSTMAP_2D_COMMUNICATION_LAYER_H_
#define COSTMAP_2D_COMMUNICATION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

#include <racecar_communication/FootprintsCombined.h>

#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>

#include <costmap_2d/footprint.h>

#include <geometry_msgs/Point.h>


namespace costmap_2d
{

class CommunicationLayer : public CostmapLayer
{
public:
  CommunicationLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D
  }

  virtual ~CommunicationLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  /*
   * @brief  A callback to handle FootprintsCombined messages
   * @param message The message returned from a message notifier
   */
  void FPCombinedMsgsCallback(const racecar_communication::FootprintsCombined& message);
  /*
   * @brief Clears the costmap layer of any previously communicated footprints
   */
  void clearPreviousFootprints();
  /*
   * @brief Marks in the costmap layer the most recently communicated footprints
   */
  void markCurrentFootprints();


protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  std::string global_frame_;  //@brief The global frame for the costmap


private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  ros::Subscriber comm_sub;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  double curr_min_x_, curr_min_y_, curr_max_x_, curr_max_y_;
  double safety_clearing;  //@brief Extra meters cleared around footprints for safety
  racecar_communication::FootprintsCombined lastFootprintsCombined; //@brief Latest array of communicated footprints
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_COMMUNICATION_LAYER_H_
