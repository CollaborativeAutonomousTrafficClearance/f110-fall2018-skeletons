#include <costmap_2d/communication_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::CommunicationLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;

namespace costmap_2d
{

CommunicationLayer::~CommunicationLayer()
{
    if (dsrv_)
        delete dsrv_;
}


void CommunicationLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  rolling_window_ = layered_costmap_->isRolling();

  // Specify the default cost of the costmap layer cells to be NO_INFORMATION
  default_value_ = NO_INFORMATION;

  // Match the costmap layer size to that of the master grid
  CommunicationLayer::matchSize();

  // Initialization of the min and max of last x and y bounds as well as the safety clearing variable
  last_min_x_ = std::numeric_limits<float>::max();
  last_min_y_ = std::numeric_limits<float>::max(); 
  last_max_x_ = -std::numeric_limits<float>::max();
  last_max_y_ = -std::numeric_limits<float>::max();
  safety_clearing = 1.5;

  // Print global frame
  global_frame_ = layered_costmap_->getGlobalFrameID();
  ROS_INFO("Global Frame: %s", global_frame_.c_str());

  // Get the topic that we'll subscribe to from the parameter server
  std::string topic, data_type;
  nh.param("communication_topic", topic, std::string(""));
  ROS_INFO("Subscribed to Topic: %s", topic.c_str());

  // Verify the topic data type
  nh.param("communication_data_type", data_type, std::string("FootprintsCombined"));
  if (!(data_type == "FootprintsCombined"))
  {
    ROS_FATAL("Only topics that use FootprintsCombined msgs are currently supported");
    throw std::runtime_error("Only topics that use FootprintsCombined msgs are currently supported");
  }
  else
  {
    comm_sub = g_nh.subscribe(topic, 50, &CommunicationLayer::FPCombinedMsgsCallback, this);  
    ROS_INFO("Created subscriber to: %s", topic.c_str());
  }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}


void CommunicationLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &CommunicationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void CommunicationLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}


void CommunicationLayer::FPCombinedMsgsCallback(const racecar_communication::FootprintsCombined& message)
{

  // Upate the value of the latest array of footprints
  lastFootprintsCombined = message;
}

void CommunicationLayer::markCurrentFootprints()
{

  // Initialize the current min and max of x and y
  curr_min_x_ = std::numeric_limits<float>::max();
  curr_min_y_ = std::numeric_limits<float>::max();
  curr_max_x_ = -std::numeric_limits<float>::max();
  curr_max_y_ = -std::numeric_limits<float>::max();

  // For each of the current footprints
  for (int i = 0; i < lastFootprintsCombined.footprints.size(); i++) 
  { 
    // Change from polygon to point vector
    std::vector<geometry_msgs::Point> footprintPoints = toPointVector(lastFootprintsCombined.footprints[i]);


    if (!footprintPoints.empty())
    {
      // Calculate the current min and max of x and y to use in updating the bounds
      double tmp_min_x, tmp_max_x, tmp_min_y, tmp_max_y;

      if (footprintPoints[0].x > getOriginX() + getSizeInMetersX()) {
        continue;
      }

      if (footprintPoints[2].x < getOriginX()) {
        continue;
      }

      if (footprintPoints[0].y > getOriginY() + getSizeInMetersY()) {
        continue;
      }

      if (footprintPoints[2].y < getOriginY()) {
        continue;
      }

      footprintPoints[0].x = std::min(std::max((getOriginX()), footprintPoints[0].x), getOriginX() + getSizeInMetersX());
      footprintPoints[1].x = std::min(std::max((getOriginX()), footprintPoints[1].x), getOriginX() + getSizeInMetersX());
      footprintPoints[2].x = std::min(std::max((getOriginX()), footprintPoints[2].x), getOriginX() + getSizeInMetersX());
      footprintPoints[3].x = std::min(std::max((getOriginX()), footprintPoints[3].x), getOriginX() + getSizeInMetersX());

      footprintPoints[0].y = std::min(std::max((getOriginY()), footprintPoints[0].y), getOriginY() + getSizeInMetersY());
      footprintPoints[1].y = std::min(std::max((getOriginY()), footprintPoints[1].y), getOriginY() + getSizeInMetersY());
      footprintPoints[2].y = std::min(std::max((getOriginY()), footprintPoints[2].y), getOriginY() + getSizeInMetersY());
      footprintPoints[3].y = std::min(std::max((getOriginY()), footprintPoints[3].y), getOriginY() + getSizeInMetersY());

      

      tmp_min_x = std::min(footprintPoints[0].x, footprintPoints[1].x);
      tmp_min_x = std::min(tmp_min_x, footprintPoints[2].x);
      tmp_min_x = std::min(tmp_min_x, footprintPoints[3].x);
      curr_min_x_ = std::min(tmp_min_x, curr_min_x_);
      //curr_min_x_ = std::max((getOriginX()), curr_min_x_);

      tmp_min_y = std::min(footprintPoints[0].y, footprintPoints[1].y);
      tmp_min_y = std::min(tmp_min_y, footprintPoints[2].y);
      tmp_min_y = std::min(tmp_min_y, footprintPoints[3].y);
      curr_min_y_ = std::min(tmp_min_y, curr_min_y_);
      //curr_min_y_ = std::max((getOriginY()), curr_min_y_);

      tmp_max_x = std::max(footprintPoints[0].x, footprintPoints[1].x);
      tmp_max_x = std::max(tmp_max_x, footprintPoints[2].x);
      tmp_max_x = std::max(tmp_max_x, footprintPoints[3].x);
      curr_max_x_ = std::max(tmp_max_x, curr_max_x_);
      //curr_max_x_ = std::min((getOriginX() + getSizeInMetersX()), curr_max_x_);

      tmp_max_y = std::max(footprintPoints[0].y, footprintPoints[1].y);
      tmp_max_y = std::max(tmp_max_y, footprintPoints[2].y);
      tmp_max_y = std::max(tmp_max_y, footprintPoints[3].y);
      curr_max_y_ = std::max(tmp_max_y, curr_max_y_);
      //curr_max_y_ = std::min((getOriginY() + getSizeInMetersY()), curr_max_y_);

      // Set the costmap layer cost of the footprint to LETHAL
      setConvexPolygonCost(footprintPoints, costmap_2d::LETHAL_OBSTACLE);
    }
  }
}


void CommunicationLayer::clearPreviousFootprints()
{
  // Specify the area to be cleared in the clearingVector
  std::vector<geometry_msgs::Point> clearingVector;
  geometry_msgs::Point pt1, pt2, pt3, pt4;

  pt1.x = last_min_x_;
  pt1.y = last_min_y_;

  pt2.x = last_min_x_;
  pt2.y = last_max_y_;

  pt3.x = last_max_x_;
  pt3.y = last_max_y_;

  pt4.x = last_max_x_;
  pt4.y = last_min_y_;

  clearingVector.push_back(pt1);
  clearingVector.push_back(pt2);
  clearingVector.push_back(pt3);
  clearingVector.push_back(pt4);
  
  if(rolling_window_) {
    resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());
  }
  else {
    // Set the costmap layer cost of any previous footprints to NO_INFORMATION
    setConvexPolygonCost(clearingVector, costmap_2d::NO_INFORMATION);
  }
}


void CommunicationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  // Return if this layer is not enabled
  if (!enabled_)
    return;

  // Upate origin if the layerd costmap is rolling
  if (rolling_window_)
  {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // Clear and mark the appropriate cells in this layer's costmap in order to be able to calculate the required bounds
  clearPreviousFootprints();
  markCurrentFootprints();

  // Include the cleared area inside the bounds
  *min_x = std::min(last_min_x_, *min_x);
  *min_y = std::min(last_min_y_, *min_y);

  *max_x = std::max(last_max_x_, *max_x);
  *max_y = std::max(last_max_y_, *max_y);

  // Include the marked area inside the bounds
  *min_x = std::min(curr_min_x_, *min_x);
  *min_y = std::min(curr_min_y_, *min_y);

  *max_x = std::max(curr_max_x_, *max_x);
  *max_y = std::max(curr_max_y_, *max_y);

  ROS_INFO("Communication Layer::Bound Includes clearing_x: [%f,%f] clearing_y: [%f,%f] marking_x: [%f,%f] marking_y: [%f,%f]", last_min_x_, last_max_x_, last_min_y_, last_max_y_, curr_min_x_, curr_max_x_, curr_min_y_, curr_max_y_);
}


void CommunicationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  // Return if this layer is not enabled
  if (!enabled_)
    return;

  // Update the cost of bounded area of the master layer using the valid costs in the costmap layer
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);

  // Update the values of the last min and max of x and y bounds
  last_min_x_ = curr_min_x_ - safety_clearing;
  last_min_y_ = curr_min_y_ - safety_clearing;

  last_max_x_ = curr_max_x_ + safety_clearing;
  last_max_y_ = curr_max_y_ + safety_clearing;
}


void CommunicationLayer::matchSize()
{
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

  resizeMap(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), costmap->getResolution(), costmap->getOriginX(), costmap->getOriginY());
}


void CommunicationLayer::activate()
{
  onInitialize();
}


void CommunicationLayer::deactivate()
{
  comm_sub.shutdown();
}


void CommunicationLayer::reset()
{
    deactivate();
    current_ = true;
    activate();
}

}  // namespace costmap_2d
