#include <cmath>
#include "mppi_social_critics/social_critic.hpp"
#include <math.h>
#include <limits>
namespace mppi::critics

{

void SocialCritic::initialize()
{   

  using std::placeholders::_1;
  auto node = parent_.lock();
  people_sub_ = node->create_subscription<people_msgs::msg::People>(
        "people", rclcpp::SensorDataQoS(),std::bind(&SocialCritic::peopleCallback,this,_1));
  pooling_timer = node->create_wall_timer(std::chrono::milliseconds(50),std::bind(&SocialCritic::costmapElaboration,this));

  /*odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&SocialCritic::odometryCallback, this, _1));*/
  /*laser_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&SocialCritic::laserCallback,this,_1));
  costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "local_costmap/costmap", rclcpp::SensorDataQoS(),
        std::bind(&SocialCritic::costmapCallback, this,_1));*/
  points_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "sfm/markers/obstacle_points",0);
  points_pub_ ->on_activate();
  /*costmap_points_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "sfm/markers/costmap_points",0);
  costmap_points_pub_ ->on_activate();*/
  agents_points_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "sfm/markers/agents_points",0);
  agents_points_pub_ ->on_activate();
  
  std::chrono::duration<int> buffer_timeout(1);
  auto getParam = parameters_handler_ ->getParamGetter(name_);
  getParam(pooling_size_, "pooling_size",5);
  getParam(person_radius_,"person_radius",0.5);
  getParam(power_,"cost_power",1);
  getParam(social_weight_,"social_weight",3000.0);
  getParam(step_granularity,"step_grouping", 12);
  getParam(FOV,"field_of_view", 90.0);
  getParam(laser_grouping,"laser_filter",4);
  getParam(max_robo_agent_x,"max_distance_robo_agent_x",3.5);
  getParam(max_robo_agent_y,"max_distance_robo_agent_y",3.5);
  getParam(laser_cutoff,"laser_distance_cut_off", 3.5);
  getParam(global_frame_,"global_frame",std::string("map"));
  RCLCPP_INFO_ONCE(logger_,"SocialCritic instantiated with %d power,%f weight,%d steps skipped,%f degrees FOV",power_,
  social_weight_,step_granularity,FOV);
  
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);
  getParentParam(time_steps_,"time_steps",60);
  getParentParam(model_dt_,"model_dt",0.05);
  // Initialize agents' vector, starting with the robot
  agents_.resize(1);
  agents_[0].desiredVelocity = 0.5f;
  agents_[0].radius = 0.35f;
  agents_[0].cyclicGoals = false;
  agents_[0].teleoperated = true;
  agents_[0].groupId = -1;
}
/*void SocialCritic::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_)
  {
    std::vector<utils::Vector2d> obstacle_points;

    if (!local_costmap_)
    {
      RCLCPP_WARN(rclcpp::get_logger("ObstacleExtractor"), "Costmap is not initialized");
    }

    int width = local_costmap_->info.width;
    int height = local_costmap_->info.height;
    double resolution = local_costmap_->info.resolution;
    double origin_x = local_costmap_->info.origin.position.x;
    double origin_y = local_costmap_->info.origin.position.y;
    std::vector<std::vector<int8_t>> grid(height, std::vector<int8_t>(width));
    RCLCPP_INFO(logger_,"Map is %d wide and %d high, with %f resolution and %f,%f origin",width,height,resolution,origin_x,origin_y);
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            int index = row * width + col;  // Calculate index in flattened array
            int8_t value = local_costmap_->data[index];
            grid[row][col] = value;
            // Count cell types
            }
        }
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        //RCLCPP_INFO(logger_,"here is index n. %d",index);
        if (grid[x][y] > 95)
        {
          utils::Vector2d point(origin_x + x * resolution,
                            origin_y + y * resolution);
          obstacle_points.push_back(point);
        }
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("ObstacleExtractor"), "Extracted %zu obstacle points", obstacle_points.size());
    publish_costmap_points(obstacle_points);
  }*/
/*void SocialCritic::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr laser) {
  if (!odom_received_)
    return;
  laser_received_ = true;
  // last_laser_ = time_;
  odom_mutex_.lock();
  builtin_interfaces::msg::Time t = base_odom_.header.stamp;
  odom_mutex_.unlock();

  RCLCPP_INFO_ONCE(logger_, "laser received");


  std::vector<utils::Vector2d> points;
  float angle = laser->angle_min;
  RCLCPP_INFO_ONCE(logger_,"angle is %f",angle);
  const unsigned int laser_grouping_ = laser_grouping;
  unsigned int laser_tot = laser->ranges.size()/laser_grouping_;
  std::vector<float> ranges_vec(laser_grouping_);

  for (unsigned int i = 0; i < laser_tot; i++) {
    for (unsigned int j = laser_grouping_; j>0; j--){
      ranges_vec[j-1] = laser->ranges[laser_grouping_*i+j-1];
    }
    auto dist_las_point = *std::min_element(std::begin(ranges_vec),std::end(ranges_vec));

    if (!std::isnan(dist_las_point) && std::isfinite(dist_las_point) &&
        dist_las_point < laser_cutoff) {

      utils::Vector2d point(dist_las_point * cos(angle+laser->angle_increment*laser_grouping_/2),
                            dist_las_point * sin(angle+laser->angle_increment*laser_grouping_/2));
      points.push_back(point);
    }
    angle += laser->angle_increment*laser_grouping_;
  }

  if (points.empty()) {
    //RCLCPP_WARN(logger_, "laser points are empty!");
    obs_mutex_.lock();
    obstacles_ = points;
    obs_mutex_.unlock();
    return;
  }
  getOdom(base_odom_);
  nav_msgs::msg::Odometry robot_odom = base_odom_;
  
  // Transform points to
  // controller_frame_ if necessary
  float shift_x = robot_odom.pose.pose.position.x;
  float shift_y = robot_odom.pose.pose.position.y;
  float angle_robot  =tf2::getYaw(robot_odom.pose.pose.orientation);
  if (laser->header.frame_id != global_frame_) {
    geometry_msgs::msg::PointStamped out;
    // builtin_interfaces::msg::Time t_las = time_;
    laser_time_ = laser->header.stamp;
    for (unsigned int i = 0; i < points.size(); i++) {
      geometry_msgs::msg::PointStamped in;
      in.header.frame_id = laser->header.frame_id;
      in.header.stamp = laser_time_;
      in.point.x = points[i].getX();
      in.point.y = points[i].getY();
      in.point.z = 0.0;
      // i did not have access to tf_buffer here so i transformed the points in the global frame using coordinates of the robot
      points[i].setX(shift_x+in.point.x*std::cos(angle_robot)-in.point.y*std::sin(angle_robot));
      points[i].setY(shift_y+in.point.y*std::cos(angle_robot)+in.point.x*std::sin(angle_robot));
      // important to do this to have the correct obstacles for the nearby agents
    }
  }
  
  // Now check if the points
  // belong to a person or an
  // dynamic obstacle

  // transform people positions to
  // rcontroller frame
  
  people_mutex_.lock();
  people_msgs::msg::People people = people_;
  people_mutex_.unlock();

  std::vector<geometry_msgs::msg::Point> people_points;
  if (!people.people.empty() &&
      people.header.frame_id != global_frame_) {
    geometry_msgs::msg::PointStamped person_point;
    person_point.header = people.header;

    for (auto person : people.people) {
      person_point.point = person.position;
      // person_point.header.stamp = time_; // node_->get_clock()->now();
      person_point.header.frame_id = people.header.frame_id;

      try {
        geometry_msgs::msg::PointStamped p_point = tf_buffer_->transform(
            person_point, global_frame_);
        people_points.push_back(p_point.point);
        RCLCPP_INFO(logger_,"person point transformed");
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(logger_,
                    "Could NOT transform "
                    "person point to %s: "
                    "%s",
                    global_frame_.c_str(), ex.what());
        return;
      }
    }
  } else if (!people.people.empty()) {
    for (auto person : people.people) {
      people_points.push_back(person.position);
    }
  }
  // Remove the points in the
  // people radius
  if (!people_points.empty()) {
    std::vector<utils::Vector2d> points_aux;
    for (utils::Vector2d p : points) {
      bool remove = false;
      for (auto person : people_points) {
        float dx = p.getX() - person.x;
        float dy = p.getY() - person.y;
        float d = std::hypotf(dx, dy);
        if (d <= 0.3f) {
          remove = true;
          break;
        }
      }
      if (!remove)
        points_aux.push_back(p);
    }
    points.clear();
    points = points_aux;
  }
  obs_mutex_.lock();
  obstacles_ = points;
  obs_mutex_.unlock();
  publish_obstacle_points(points);}
*/
void SocialCritic::publish_obstacle_points(
    const std::vector<utils::Vector2d> &points) {
  // function to publish the points to be used for the obstacles for the agents
  visualization_msgs::msg::Marker m;
  m.header.frame_id = global_frame_; // robot_frame_
  // m.header.stamp = time_;
  m.header.stamp = laser_time_;
  m.ns = "sfm_obstacle_points";
  m.type = visualization_msgs::msg::Marker::POINTS;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.15;
  m.scale.y = 0.15;
  m.scale.z = 0.15;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.id = 1000;
  m.lifetime = rclcpp::Duration::from_seconds(0.3);
  for (utils::Vector2d p : points) {
    geometry_msgs::msg::Point pt;
    pt.x = p.getX();
    pt.y = p.getY();
    pt.z = 0.2;
    m.points.push_back(pt);
  }
  points_pub_->publish(m);
}
/*void SocialCritic::publish_costmap_points(
    const std::vector<utils::Vector2d> &points) {
  // function to publish the points to be used for the obstacles for the agents
  visualization_msgs::msg::Marker m;
  m.header.frame_id = global_frame_; // robot_frame_
  // m.header.stamp = time_;
  m.header.stamp = laser_time_;
  m.ns = "sfm_costmap_points";
  m.type = visualization_msgs::msg::Marker::POINTS;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.15;
  m.scale.y = 0.15;
  m.scale.z = 0.15;
  m.color.r = 0.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.id = 1000;
  m.lifetime = rclcpp::Duration::from_seconds(0.3);
  for (utils::Vector2d p : points) {
    geometry_msgs::msg::Point pt;
    pt.x = p.getX();
    pt.y = p.getY();
    pt.z = 0.2;
    m.points.push_back(pt);
  }
  costmap_points_pub_->publish(m);
}*/
void SocialCritic::publish_agents_points(const std::vector<sfm::Agent> &agents){
  //function to publish the points of the agents used in computations
  visualization_msgs::msg::Marker a;
  a.header.frame_id = global_frame_; // robot_frame_
  // a.header.stamp = time_;
  a.header.stamp = people_time_;
  a.ns = "used_agents_points";
  a.type = visualization_msgs::msg::Marker::POINTS;
  a.action = visualization_msgs::msg::Marker::ADD;
  a.pose.orientation.x = 0.0;
  a.pose.orientation.y = 0.0;
  a.pose.orientation.z = 0.0;
  a.pose.orientation.w = 1.0;
  a.scale.x = 0.25;
  a.scale.y = 0.25;
  a.scale.z = 0.25;
  a.color.r = 0.0;
  a.color.g = 0.0;
  a.color.b = 1.0;
  a.color.a = 1.0;
  a.id = 1000;
  a.lifetime = rclcpp::Duration::from_seconds(0.3);
  // printf("Published Obstacles: ");
  for (size_t i = 0;i < agents.size();++i) {
    geometry_msgs::msg::Point pt;
    pt.x = agents[i].position.getX();
    pt.y = agents[i].position.getY();
    // printf("x: %.2f, y: %.2f -", pt.x, pt.y);
    pt.z = 0.2;
    a.points.push_back(pt);
  }
  // printf("\n");
  agents_points_pub_->publish(a);
}
/*void SocialCritic::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {


  RCLCPP_INFO_ONCE(logger_, "Odom received");
  odom_received_ = true;

  odom_mutex_.lock();
  base_odom_ = *odom;
  odom_mutex_.unlock();

  agents_mutex_.lock();
  sfm::Agent agent = agents_[0];
  agents_mutex_.unlock();

  agent.position.set(odom->pose.pose.position.x, odom->pose.pose.position.y);
  agent.yaw =
      utils::Angle::fromRadian(tf2::getYaw(odom->pose.pose.orientation));

  agent.linearVelocity =
      std::sqrt(odom->twist.twist.linear.x * odom->twist.twist.linear.x +
                odom->twist.twist.linear.y * odom->twist.twist.linear.y);
  agent.angularVelocity = odom->twist.twist.angular.z;

  // The velocity in the odom messages is in the robot local frame!!!
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = odom->twist.twist.linear.x;
  velocity.y = odom->twist.twist.linear.y;

  agent.velocity.set(velocity.x, velocity.y);
  //std::vector<sfm::Agent> obstacle_agents;
  // Update agent[0] (the robot) with odom.
  agents_mutex_.lock();
  agents_[0] = agent;
  //obstacle_agents = agents_[1:end]; 
  std::vector<sfm::Agent> obstacle_agents(agents_.begin() + 1, agents_.end());
  agents_mutex_.unlock();
  RCLCPP_DEBUG(logger_,"Agent updated, %d",obstacle_agents[1].id);

  std::vector<utils::Vector2d> obs_points;
  std::vector<utils::Vector2d> agent_positions;
  // check if the agent is in the local costmap and update the obstacles
  for (auto &new_agent : obstacle_agents) {
    // skip the robot
    if (new_agent.id == 0) {
      continue;
    }
    RCLCPP_DEBUG(
      logger_, "Agent %d: (%f, %f)", new_agent.id, new_agent.position.getX(),
      new_agent.position.getY());
    unsigned int mx, my;
    if (!costmap_->worldToMap(new_agent.position.getX(), new_agent.position.getY(), mx, my)) {
      RCLCPP_DEBUG(
        logger_, "Agent %d is not in the local costmap", new_agent.id);
      continue;
    }
    // Check if the agent is in the fov of the robot
    double dx = new_agent.position.getX() - agent.position.getX();
    double dy = new_agent.position.getY() - agent.position.getY();
    double angle = atan2(dy, dx);
    double angle_diff = angle - tf2::getYaw(odom->pose.pose.orientation);
    angle_diff = atan2(sin(angle_diff), cos(angle_diff));
    //RCLCPP_INFO(logger_,"Angle diff is %f",angle_diff);

    if (fabs(angle_diff) > FOV/360*M_PI) {
      RCLCPP_DEBUG(
        logger_, "Agent %d is not in the fov of the robot", agent.id);
      continue;
    }
    agent_positions.push_back(agent.position);
    obstacle_agents.push_back(agent);
  }
  std::vector<geometry_msgs::msg::Point> obstacles;
  // Select a coarser resolution for the local costmap and retrieve the obstacles
  // select a resolution that is a multiple of the global costmap resolution
  nav2_costmap_2d::Costmap2D costmap = maxPooling(costmap_, pooling_size_);
  double resolution = costmap.getResolution();
  int people_radius_cell = person_radius_ / resolution + 1; 
  // selec n points in the local costmap and check if they are obstacles
  RCLCPP_DEBUG(
    logger_, "Costmap resolution: %f", resolution);
  for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++) {
      unsigned int mx, my;
      RCLCPP_DEBUG(
        logger_, "Cell: (%d, %d)", i, j);
      auto index = costmap.getIndex(i, j);
      costmap.indexToCells(index, mx, my);
      if (costmap.getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        double wx, wy;
        // check if the obstacle is not a person
        bool is_person = false;
        for (auto &pos : agent_positions) {
          unsigned int px, py;
          costmap.worldToMap(pos.getX(), pos.getY(), px, py);
          if (hypotf(mx -  px, my - py ) < people_radius_cell) {
            is_person = true;
            break;
          }
        }
        if (is_person) {
          continue;
        }
        costmap.mapToWorld(mx, my, wx, wy);
        obstacles.push_back(geometry_msgs::msg::Point());
        obstacles.back().x = wx;
        obstacles.back().y = wy;
        obs_points.push_back(utils::Vector2d(wx, wy));
      }
    }
  }

  RCLCPP_DEBUG(
    logger_, "Number of obstacles: %lu", obs_points.size());
  publish_obstacle_points(obs_points);
}*/
void SocialCritic::peopleCallback(const people_msgs::msg::People::SharedPtr people) {

  //if (!running_ || !odom_received_)
  // return;

  RCLCPP_INFO_ONCE(logger_, "People received");
  people_mutex_.lock();
  people_ = *people;
  people_mutex_.unlock();
  
  odom_mutex_.lock();
  //builtin_interfaces::msg::Time t = base_odom_.header.stamp;
  //t_ = t;
  odom_mutex_.unlock();
  RCLCPP_INFO_ONCE(logger_,"Time acquired");
  std::vector<sfm::Agent> agents;
  
  // check if people are not in odom frame
  geometry_msgs::msg::PoseStamped ps;
  
  for (unsigned i = 0; i < people->people.size(); i++) {
    sfm::Agent ag;
    ag.id = std::stoi(people->people[i].tags[0]);
    ag.groupId = std::stoi(people->people[i].tags[1]);

    ps.header.frame_id = people->header.frame_id;
    // builtin_interfaces::msg::Time t = time_;
    people_time_ = people->header.stamp;
    ps.header.stamp = people_time_;
    ps.pose.position = people->people[i].position;
    ps.pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, people->people[i].position.z);
    ps.pose.orientation = tf2::toMsg(quat);
    /*if (people->header.frame_id != global_frame_) {
      geometry_msgs::msg::PoseStamped p;
      try {
        p = tf_buffer_->transform(ps, global_frame_);
        ps = p;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(logger_, "PeopleCallback. No transform %s",
                    ex.what());
        return;
      }
    }*/
    ag.position.set(ps.pose.position.x, ps.pose.position.y);

    geometry_msgs::msg::Vector3 velocity;
    velocity.x = people->people[i].velocity.x;
    velocity.y = people->people[i].velocity.y;
    velocity.z = 0.0;

    /*geometry_msgs::msg::Vector3 localV = SocialCritic::transformVector(
        velocity, t, people->header.frame_id, global_frame_);*/

    ag.velocity.set(velocity.x, velocity.y);
    ag.linearVelocity = ag.velocity.norm();
    if (fabs(ag.linearVelocity) < 0.09)
      ag.yaw.setRadian(tf2::getYaw(ps.pose.orientation));
    else
      ag.yaw = utils::Angle::fromRadian(
          atan2(ag.velocity.getY(), ag.velocity.getX()));
    ag.angularVelocity = people->people[i].velocity.z;
    ag.radius = 0.3f;
    ag.teleoperated = false;

    // The SFM requires a local goal for each agent. We will assume that the
    // goal for people depends on its current velocity
    ag.goals.clear();
    sfm::Goal naiveGoal;
    utils::Vector2d v =
        ag.position + time_steps_*model_dt_* ag.velocity;
    naiveGoal.center.set(v.getX(), v.getY());
    naiveGoal.radius = 0.3f;
    ag.goals.push_back(naiveGoal);
    ag.desiredVelocity = 0.4f;
    agents.push_back(ag);
    RCLCPP_INFO_ONCE(logger_,"agents updated");
  }

  // Fill the obstacles of the agents
  obs_mutex_.lock();
  std::vector<utils::Vector2d> obs_points = obstacles_;
  obs_mutex_.unlock();
  for (unsigned int i = 0; i < agents.size(); i++) {
    agents[i].obstacles1.clear();
    agents[i].obstacles1 = obs_points;
  }

  agents_mutex_.lock();
  agents_.resize(people->people.size() + 1);
  //agents_[0].obstacles1 = obs_points;
  for (unsigned int i = 1; i < agents_.size(); i++)
    agents_[i] = agents[i - 1];
  agents_mutex_.unlock();
}

/*void SocialCritic::getOdom(nav_msgs::msg::Odometry &base_odom) {
  // obtaining information about odometry to be used to put obstacles points in correct frame
  odom_mutex_.lock();
  base_odom = base_odom_;
  odom_mutex_.unlock();
}*/

std::vector<sfm::Agent> SocialCritic::getAgents() {
  // obtaining informations about agents to be used in scoring function
  agents_mutex_.lock();
  std::vector<sfm::Agent> agents_calc = agents_;
  agents_mutex_.unlock();
  return agents_calc;
}
double SocialCritic::computeSocialWork(const std::vector<sfm::Agent> &agents){
      // The social work of the robot
  double wr = agents[0].forces.socialForce.norm();

  // Compute the social work provoked by the robot in the other agents
  std::vector<sfm::Agent> agent_robot;
  agent_robot.push_back(agents[0]);
  double wp = 0.0;
  for (unsigned int i = 1; i < agents.size(); i++) {
    sfm::Agent agent = agents[i];
    sfm::SFM.computeForces(agent, agent_robot);
    wp += agent.forces.socialForce.norm();
  }
  return wr+wp;
}
/*
geometry_msgs::msg::Vector3
SocialCritic::transformVector(geometry_msgs::msg::Vector3 &vector,
                                    builtin_interfaces::msg::Time t,
                                    std::string from, std::string to) {
  geometry_msgs::msg::Vector3 nextVector;

  geometry_msgs::msg::Vector3Stamped v;
  geometry_msgs::msg::Vector3Stamped nv;

  v.header.frame_id = from;
  v.header.stamp = t; // node_->get_clock()->now();
  v.vector.x = vector.x;
  v.vector.y = vector.y;
  v.vector.z = 0.0;

  try {
    nv = tf_buffer_->transform(v, to);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(
        logger_,
        "TransformVector. No transform from %s frame to %s frame. Ex: %s",
        from.c_str(), to.c_str(), ex.what());
  }

  nextVector.x = nv.vector.x;
  nextVector.y = nv.vector.y;
  nextVector.z = 0.0;

  return nextVector;
}
*/
void SocialCritic::costmapElaboration(){
  std::vector<utils::Vector2d> obs_points;
  /*std::vector<sfm::Agent> total_agents = getAgents();
  std::vector<sfm::Agent> obstacle_agents;
  
  std::vector<utils::Vector2d> agent_positions;
  utils::Vector2d robot_vector;
  utils::Vector2d distance_vector;
  double robo_angle = tf2::getYaw(data.state.pose.pose.orientation);
  robot_vector.set(robo_angle.cos(),robo_angle.sin());*/
  RCLCPP_DEBUG(logger_, "Costmap elaboration");
  // check if the agent is in the local costmap and update the obstacles
  /*
  for (auto &new_agent : total_agents) {
    // skip the robot
    if (new_agent.id == total_agents[0].id) {
      continue;
    }
    RCLCPP_INFO(
      logger_, "Agent %d: (%f, %f)", new_agent.id, new_agent.position.getX(),
      new_agent.position.getY());
    unsigned int mx, my;
    if (!costmap_->worldToMap(new_agent.position.getX(), new_agent.position.getY(), mx, my)) {
      RCLCPP_INFO(
        logger_, "Agent %d is not in the local costmap", new_agent.id);
      continue;
    }
    // Check if the agent is in the fov of the robot
    double dx = new_agent.position.getX() - data.state.pose.pose.position.x;
    double dy = new_agent.position.getY() - data.state.pose.pose.position.y;
    distance_vector.set(dx, dy);
    utils::Angle robo_agent_angle = distance_vector.angleTo(robot_vector);
    double value_ang = std::abs(robo_agent_angle.toRadian());
    RCLCPP_INFO(logger_,"Angle is %f",value_ang);
    double angle = atan2(dy, dx);
    double angle_diff = angle - total_agents[0].yaw.setRadian;
    RCLCPP_INFO(logger_,"Angle are:%f,%f",angle,total_agents[0].yaw.setRadian());
    angle_diff = atan2(sin(angle_diff), cos(angle_diff));
    RCLCPP_INFO(logger_,"Angle diff is %f",angle_diff);

    if (value_ang > FOV/360*M_PI) {
      RCLCPP_INFO(
        logger_, "Agent %d is not in the fov of the robot", new_agent.id);
      continue;
    }
    agent_positions.push_back(new_agent.position);
    obstacle_agents.push_back(new_agent);
  }*/
  std::vector<geometry_msgs::msg::Point> obstacles;
  // Select a coarser resolution for the local costmap and retrieve the obstacles
  // select a resolution that is a multiple of the global costmap resolution
  nav2_costmap_2d::Costmap2D costmap = maxPooling(costmap_, pooling_size_);
  double resolution = costmap.getResolution();
  //int people_radius_cell = person_radius_ / resolution + 1; 
  // selec n points in the local costmap and check if they are obstacles
  RCLCPP_DEBUG(
    logger_, "Costmap resolution: %f", resolution);
  for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++) {
      unsigned int mx, my;
      RCLCPP_DEBUG(
        logger_, "Cell: (%d, %d)", i, j);
      auto index = costmap.getIndex(i, j);
      costmap.indexToCells(index, mx, my);
      if (costmap.getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        double wx, wy;
        // check if the obstacle is not a person
        /*bool is_person = false;
        for (auto &pos : agent_positions) {
          unsigned int px, py;
          costmap.worldToMap(pos.getX(), pos.getY(), px, py);
          if (hypotf(mx -  px, my - py ) < people_radius_cell) {
            is_person = true;
            RCLCPP_INFO(logger_,"Person detected, skipping");
            break;
          }
        }
        if (is_person) {
          continue;
        }*/
        costmap.mapToWorld(mx, my, wx, wy);
        obstacles.push_back(geometry_msgs::msg::Point());
        obstacles.back().x = wx;
        obstacles.back().y = wy;
        obs_points.push_back(utils::Vector2d(wx, wy));
      }
    }
  }

  RCLCPP_DEBUG(
    logger_, "Number of obstacles: %lu", obs_points.size());
  obs_mutex_.lock();
  obstacles_ = obs_points;
  obs_mutex_.unlock();
  //publish_obstacle_points(obs_points);
}


void SocialCritic::score(CriticData & data){
  using xt::evaluation_strategy::immediate;
  if (!enabled_) {
    RCLCPP_INFO_ONCE(logger_,"agent critic disabled!");
    return;
    
  }
  std::vector<sfm::Agent> myagents = getAgents();
  agents_mutex_.lock();
  sfm::Agent agent = myagents[0];
  agents_mutex_.unlock();

  agent.position.set(data.state.pose.pose.position.x, data.state.pose.pose.position.y);
  agent.yaw =
      utils::Angle::fromRadian(tf2::getYaw(data.state.pose.pose.orientation));

  agent.linearVelocity =
      std::sqrt(data.state.speed.linear.x * data.state.speed.linear.x +
                data.state.speed.linear.y * data.state.speed.linear.y);
  agent.angularVelocity = data.state.speed.angular.z;

  // The velocity in the odom messages is in the robot local frame!!!
  geometry_msgs::msg::Vector3 velocity;
  velocity.x = data.state.speed.linear.x;
  velocity.y = data.state.speed.linear.y;

  agent.velocity.set(velocity.x, velocity.y);
  //std::vector<sfm::Agent> obstacle_agents;
  // Update agent[0] (the robot) with odom.
  agents_mutex_.lock();
  myagents[0] = agent;
  //obstacle_agents = myagents;
  agents_mutex_.unlock();
  /*
  std::vector<utils::Vector2d> obs_points;
  std::vector<utils::Vector2d> agent_positions;
  // check if the agent is in the local costmap and update the obstacles
  for (auto &new_agent : obstacle_agents) {
    // skip the robot
    if (new_agent.id == 0) {
      continue;
    }
    RCLCPP_DEBUG(
      logger_, "Agent %d: (%f, %f)", new_agent.id, new_agent.position.getX(),
      new_agent.position.getY());
    unsigned int mx, my;
    if (!costmap_->worldToMap(new_agent.position.getX(), new_agent.position.getY(), mx, my)) {
      RCLCPP_DEBUG(
        logger_, "Agent %d is not in the local costmap", new_agent.id);
      continue;
    }
    // Check if the agent is in the fov of the robot
    double dx = new_agent.position.getX() - agent.position.getX();
    double dy = new_agent.position.getY() - agent.position.getY();
    double angle = atan2(dy, dx);
    double angle_diff = angle - tf2::getYaw(data.state.pose.pose.orientation);
    angle_diff = atan2(sin(angle_diff), cos(angle_diff));

    if (fabs(angle_diff) > FOV/360*M_PI) {
      RCLCPP_DEBUG(
        logger_, "Agent %d is not in the fov of the robot", agent.id);
      continue;
    }
    agent_positions.push_back(agent.position);
    obstacle_agents.push_back(agent);
  }
  std::vector<geometry_msgs::msg::Point> obstacles;
  // Select a coarser resolution for the local costmap and retrieve the obstacles
  // select a resolution that is a multiple of the global costmap resolution
  nav2_costmap_2d::Costmap2D costmap = maxPooling(costmap_, pooling_size_);
  double resolution = costmap.getResolution();
  int people_radius_cell = person_radius_ / resolution + 1; 
  // selec n points in the local costmap and check if they are obstacles
  RCLCPP_DEBUG(
    logger_, "Costmap resolution: %f", resolution);
  for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++) {
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++) {
      unsigned int mx, my;
      RCLCPP_DEBUG(
        logger_, "Cell: (%d, %d)", i, j);
      auto index = costmap.getIndex(i, j);
      costmap.indexToCells(index, mx, my);
      if (costmap.getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        double wx, wy;
        // check if the obstacle is not a person
        bool is_person = false;
        for (auto &pos : agent_positions) {
          unsigned int px, py;
          costmap.worldToMap(pos.getX(), pos.getY(), px, py);
          if (hypotf(mx -  px, my - py ) < people_radius_cell) {
            is_person = true;
            break;
          }
        }
        if (is_person) {
          continue;
        }
        costmap.mapToWorld(mx, my, wx, wy);
        obstacles.push_back(geometry_msgs::msg::Point());
        obstacles.back().x = wx;
        obstacles.back().y = wy;
        obs_points.push_back(utils::Vector2d(wx, wy));
      }
    }
  }

  RCLCPP_DEBUG(
    logger_, "Number of obstacles: %lu", obs_points.size());
  publish_obstacle_points(obs_points);*/
  std::vector<sfm::Agent> used_agents;
  if (myagents.empty()){
    RCLCPP_WARN(logger_,"no agent acquired!");
  }

  auto && social_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
  social_cost.fill(0.0f);
  float vx_i = 0.0f;
  float vy_i = 0.0f;
  const size_t num_traj = data.trajectories.x.shape(0);
  const size_t traj_len = data.trajectories.x.shape(1)/step_granularity;
  float diff_x;
  float diff_y;
  float dist_x;
  float dist_y;
  utils::Vector2d distance_vector;
  utils::Vector2d robot_vector;
  utils::Angle robo_agent_angle;
  double FOV_radians = FOV/360*M_PI;
  double value_ang;
  robot_vector.set(myagents[0].yaw.cos(),myagents[0].yaw.sin());
  //float dist_;
  short int counter_ = 1;
  used_agents.push_back(myagents[0]);

  //robot_vector.set(myagents[0].velocity.getX(),myagents[0].velocity.getY());
  for (size_t z=1;z<myagents.size();++z){
    diff_x = myagents[z].position.getX()-myagents[0].position.getX();
    diff_y = myagents[z].position.getY()-myagents[0].position.getY();
    distance_vector.set(diff_x,diff_y);
    robo_agent_angle = distance_vector.angleTo(robot_vector);
    value_ang = std::abs(robo_agent_angle.toRadian());
    //round filter area
    /*dist_x = std::pow(diff_x,2);
    dist_y = std::pow(diff_y,2);
    dist_ = std::sqrt(dist_x+dist_y);
    if (dist_ < 4.0f){
      ++counter_;
      used_agents.push_back(myagents[z]);
    }*/
    //square filter area
    dist_x= std::abs(diff_x);
    dist_y = std::abs(diff_y);
    if (dist_x<max_robo_agent_x && dist_y<max_robo_agent_y && value_ang<FOV_radians){
      ++counter_;
      used_agents.push_back(myagents[z]);
      
    }
  }
  publish_agents_points(used_agents);
  std::vector<utils::Vector2d> new_obstacles;
  for (auto &point : obstacles_){
    bool agent_point = false;
    for (auto &agent : used_agents){
      if (hypotf(point.getX()-agent.position.getX(),point.getY()-agent.position.getY())<person_radius_){
        RCLCPP_INFO(logger_,"obstacle detected");
        agent_point = true;
        break;
      }     
      }
    if (!agent_point){
        new_obstacles.push_back(point);
    }

  }
  for (unsigned int i = 0; i < used_agents.size(); i++) {
    used_agents[i].obstacles1.clear();
    used_agents[i].obstacles1 = new_obstacles;
  }
  publish_obstacle_points(new_obstacles);

  /*
  float agentstep_x = 0.0f;
  float agentstep_y = 0.0f;
  const size_t people_in_scene = used_agents.size();
  xt::xarray<float>::shape_type sh0 = {people_in_scene, traj_len};
  auto x_agents = xt::empty<float>(sh0);
  auto y_agents = xt::empty<float>(sh0);
  for (size_t k = 0;k<traj_len; ++k){
            // update agents position for this time step
        for (size_t m=0; m<people_in_scene; ++m){
            agentstep_x = used_agents[m+1].velocity.getX()*(0.05f*step_granularity);
            agentstep_y = used_agents[m+1].velocity.getY()*(0.05f*step_granularity);
            //myagents[k].position.set(myagents[k].position.getX()+agentstep_x,myagents[k].position.getY()+agentstep_y);        
            x_agents[{m,k}] = used_agents[m+1].position.getX()+agentstep_x;
            y_agents[{m,k}] = used_agents[m+1].position.getY()+agentstep_y;
        }
  }*/


  //short int wanm = used_agents.size();
  //RCLCPP_INFO(logger_,"agents used are %d",wanm);
  //RCLCPP_INFO(logger_,"robo vec is %f:%f",robot_vector.getX(),robot_vector.getY());
  //RCLCPP_INFO(logger_,"angle is %f",myagents[0].yaw.toDegree());
  //used_agents.resize(counter_);
  for (size_t i = 0; i < num_traj; ++i) {
    const auto & traj = data.trajectories;
    double social_work = 0.0;
    double social_step = 0.0;
    sfm::Goal g;
    g.center.set(traj.x(i,traj_len), traj.y(i,traj_len));
    g.radius = 0.20;
    used_agents[0].goals.clear();
    used_agents[0].goals.push_back(g);
    std::vector<sfm::Agent> updating_agents = used_agents;
    for (size_t j=0; j< traj_len; ++j){
      //compute forces for this step
      sfm::SFM.computeForces(updating_agents);
      // update agents position for this step
      sfm::SFM.updatePosition(updating_agents,0.05f*step_granularity);
      // update robot position for this time step
      updating_agents[0].position.set(traj.x(i,step_granularity*j),traj.y(i,step_granularity*j));
      updating_agents[0].yaw = utils::Angle::fromRadian(traj.yaws(i,step_granularity*j));
      // update robot velocity for this time step
      vx_i = (-traj.x(i,step_granularity*j)+traj.x(i,step_granularity*(j+1)-1))/(0.05f*step_granularity);
      vy_i= (-traj.y(i,step_granularity*j)+traj.y(i,step_granularity*(j+1)-1))/(0.05f*step_granularity);
      updating_agents[0].velocity.set(vx_i, vy_i);
      updating_agents[0].angularVelocity = (-traj.yaws(i,step_granularity*j)+traj.yaws(i,step_granularity*(j+1)-1))/(0.05f*step_granularity);
      updating_agents[0].linearVelocity = hypotf(vx_i,vy_i);
      //compute social work for this step, add it to overall work
      social_step = computeSocialWork(updating_agents);
      social_work += social_step;
      }

    
    social_cost[i] =social_work;
    //RCLCPP_INFO(logger_,"traj has social cost %f",social_cost[i]);
  }

  
  data.costs += xt::pow(social_cost*social_weight_/traj_len,power_);
}
nav2_costmap_2d::Costmap2D SocialCritic::maxPooling(nav2_costmap_2d::Costmap2D * costmap, unsigned int pooling_size)
{
  RCLCPP_DEBUG(
    logger_, "Max pooling");
  // Create a new costmap with a size reduced by pooling_size
  unsigned int size_x = costmap->getSizeInCellsX() % pooling_size == 0 ?
    costmap->getSizeInCellsX() / pooling_size :
    costmap->getSizeInCellsX() / pooling_size + 1;
  unsigned int size_y = costmap->getSizeInCellsY() % pooling_size == 0 ?
    costmap->getSizeInCellsY() / pooling_size :
    costmap->getSizeInCellsY() / pooling_size + 1;

  nav2_costmap_2d::Costmap2D new_costmap = nav2_costmap_2d::Costmap2D(
    size_x,
    size_y,
    costmap->getResolution() * pooling_size,
    costmap->getOriginX(), costmap->getOriginY(), costmap->getDefaultValue());
  // Initialize the new costmap
  new_costmap.setDefaultValue(new_costmap.getDefaultValue());
  new_costmap.resetMap(0, 0, new_costmap.getSizeInCellsX(), new_costmap.getSizeInCellsY());
  
  // Apply the max pooling
  for ( unsigned int i = 0; i < costmap->getSizeInCellsX(); i++) {
    for (unsigned int j = 0; j < costmap->getSizeInCellsY(); j++) {
      unsigned int mx, my;
      auto index = costmap->getIndex(i, j);
      costmap->indexToCells(index, mx, my);
      unsigned int new_mx = mx / pooling_size;
      unsigned int new_my = my / pooling_size;
      
      if (costmap->getCost(mx, my) > new_costmap.getCost(new_mx, new_my)) {
        new_costmap.setCost(new_mx, new_my, costmap->getCost(mx, my));
      }
    }
  } 
 

  RCLCPP_DEBUG(
    logger_, "Max pooling done");
  return new_costmap;
}


}// namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
mppi::critics::SocialCritic,
mppi::critics::CriticFunction)
