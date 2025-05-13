
#include <climits>
#include <pluginlib/class_list_macros.h>
#include <fstream>
#include "keep_out_layer.h"
#include <ros/package.h>
#include <sys/stat.h>
#include <errno.h>
#include <iomanip>
#include <json/json.h>  // Include the JSON library

PLUGINLIB_EXPORT_CLASS(costmap_2d::KeepOutLayer, costmap_2d::Layer)

namespace costmap_2d {

// 全局变量声明
std::string global_map_name;

// 获取地图名称的函数
std::string getMapName(ros::NodeHandle& nh) {
    std::string map_path;
    if (nh.getParam("map", map_path)) {
        ROS_INFO_STREAM("Map path: " << map_path);

        // 从完整路径中提取文件名
        size_t last_slash = map_path.find_last_of("/\\");
        std::string filename = (last_slash == std::string::npos) ? map_path : map_path.substr(last_slash + 1);

        // 去除文件名的后缀（.yaml）
        size_t dot_pos = filename.find_last_of(".");
        if (dot_pos != std::string::npos) {
            filename = filename.substr(0, dot_pos);  // 截取去掉扩展名后的部分
        }

        ROS_INFO_STREAM("Map filename without extension: " << filename);
        return filename;
    } else {
        ROS_WARN("Failed to get param 'map'");
        return "";
    }
}

void KeepOutLayer::onInitialize() {
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle global_nh;  // 全局命名空间的NodeHandle
    
    current_ = true;

    // 获取地图名称并赋给全局变量
    global_map_name = getMapName(global_nh);
    ROS_WARN_STREAM("[虚拟墙层] 当前地图名称: " << global_map_name);

    nh.param("enabled", enabled_, false);
    nh.param("fill_zones", fill_zones_, true);

    map_received_ = false;
    rolling_window_ = layered_costmap_->isRolling();
    setDefaultValue(NO_INFORMATION);
    zone_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("/record_zone", 1);
    map_sub_ = nh.subscribe("/map", 1, &KeepOutLayer::incomingMap, this);
    point_sub_ = nh.subscribe("/clicked_point", 1, &KeepOutLayer::incomingPoint, this);

    keep_out_zone_srv_ = nh.advertiseService("xju_zone", &KeepOutLayer::keepOutZoneSrv, this);
}

void KeepOutLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map) {
  map_received_ = false;
  while (updating_) {
    ROS_WARN_THROTTLE(1.0, "[虚拟墙层] 导入地图，同时更新代价，等待...");
  }

  resizeMap(new_map->info.width, new_map->info.height, new_map->info.resolution,
            new_map->info.origin.position.x, new_map->info.origin.position.y);
  
  ROS_INFO("[虚拟墙层] 接收到一个 %d X %d 的地图，分辨率为 %f m/pix，原点在(%.2f, %.2f)",
           size_x_, size_y_, resolution_, origin_x_, origin_y_);

  // 先清除现有的代价值
  resetMaps();
  
  // 加载虚拟墙
  if (loadVirtualWallFromFile()) {
    ROS_INFO("[虚拟墙层] 已加载虚拟墙代价地图");
  }
  
  // 设置所有区域的代价值
  setAllZonesCost();
  
  map_received_ = true;

  // 触发边界更新
  if (enabled_) {
    double min_x = origin_x_, min_y = origin_y_;
    double max_x = origin_x_ + size_x_ * resolution_;
    double max_y = origin_y_ + size_y_ * resolution_;
    addExtraBounds(min_x, min_y, max_x, max_y);
  }
}

bool KeepOutLayer::loadVirtualWallFromFile() {
  try {
    // 获取功能包路径
    std::string package_path = ros::package::getPath("costmap_plugins");
    std::string filepath = package_path + "/file/" + name_ +"_"+ global_map_name+ "_virtual_wall.json";

    // 检查文件是否存在
    std::ifstream file(filepath, std::ifstream::binary);
    if (!file.is_open()) {
      ROS_WARN("[虚拟墙层] 没有找到虚拟墙配置文件: %s", filepath.c_str());
      return false;
    }

    // 检查文件是否为空
    file.seekg(0, std::ios::end);
    if (file.tellg() == 0) {
      ROS_WARN("[虚拟墙层] 虚拟墙配置文件为空: %s", filepath.c_str());
      return false;
    }
    file.seekg(0, std::ios::beg);

    // 解析 JSON 文件
    Json::Value root;
    file >> root;
    file.close();

    // 检查 JSON 根元素是否为数组
    if (!root.isMember("walls") || !root["walls"].isArray()) {
      ROS_ERROR("[虚拟墙层] 虚拟墙配置格式错误");
      return false;
    }

    // 遍历每个虚拟墙
    for (const auto& wall : root["walls"]) {
      int num_points = wall["num_points"].asInt();
      ROS_INFO("[虚拟墙层] 读取到 %d 点", num_points);

      if (num_points < 2) {
        ROS_WARN("[虚拟墙层] 虚拟墙配置文件点数不足，(至少需要2个点， 获取到 %d)", num_points);
        continue;
      }

      // 读取所有点
      PointVector points_vector;
      for (const auto& json_point : wall["points"]) {
        geometry_msgs::Point point;
        point.x = json_point["x"].asDouble();
        point.y = json_point["y"].asDouble();
        point.z = json_point["z"].asDouble();
        ROS_INFO("[虚拟墙层] 读取到: (%.3f, %.3f, %.3f)", point.x, point.y, point.z);
        points_vector.push_back(point);
      }

      // 如果只有两个点，创建一个细长的多边形
      if (num_points == 2) {
        PointVector expanded_points;
        geometry_msgs::Point p1 = points_vector[0];
        geometry_msgs::Point p2 = points_vector[1];

        // 计算垂直于线段的单位向量
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double length = std::sqrt(dx*dx + dy*dy);
        double nx = -dy/length;  // 垂直向量
        double ny = dx/length;
        
        // 设置多边形的宽度（可以根据需要调整）
        double width = resolution_ * 2;  // 设置为地图分辨率的两倍
        
        // 创建四个角点形成矩形
        geometry_msgs::Point corner;
        
        // 第一个点的两个角
        corner.x = p1.x + nx * width/2;
        corner.y = p1.y + ny * width/2;
        corner.z = 0;
        expanded_points.push_back(corner);
        
        corner.x = p1.x - nx * width/2;
        corner.y = p1.y - ny * width/2;
        corner.z = 0;
        expanded_points.push_back(corner);
        
        // 第二个点的两个角
        corner.x = p2.x - nx * width/2;
        corner.y = p2.y - ny * width/2;
        corner.z = 0;
        expanded_points.push_back(corner);
        
        corner.x = p2.x + nx * width/2;
        corner.y = p2.y + ny * width/2;
        corner.z = 0;
        expanded_points.push_back(corner);

        points_vector = expanded_points;
      }

      // 添加虚拟墙
      uint32_t id;
      if (!addZone(points_vector, id, LETHAL_OBSTACLE)) {
        ROS_ERROR("[虚拟墙层] 从配置文件中加载虚拟墙失败");
        continue;
      }

      ROS_INFO("[虚拟墙层] 成功加载虚拟墙，包含%zu 点", points_vector.size());
    }

    return true;

  } catch (const std::exception& e) {
    ROS_ERROR("[虚拟墙层] 加载虚拟墙时出错: %s", e.what());
    return false;
  }
}

void KeepOutLayer::incomingPoint(const geometry_msgs::PointStampedConstPtr& point) {
  if (!recording_) return;

  record_zone_.emplace_back(*point);
  geometry_msgs::PolygonStamped polygon;
  polygon.header.frame_id = "map";
  polygon.header.stamp = ros::Time::now();
  geometry_msgs::Point32 point32;
  for (auto const& p : record_zone_) {
    point32.x = p.point.x;
    point32.y = p.point.y;
    point32.z = 0;
    polygon.polygon.points.emplace_back(point32);
  }
  zone_pub_.publish(polygon);
}

void KeepOutLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double* min_x, double* min_y, double* max_x, double* max_y) {
  // never update bounds of keep out layer
  if (!map_received_ || !enabled_) return;
  useExtraBounds(min_x, min_y, max_x, max_y);
  if (rolling_window_) {
    auto out_width = static_cast<int>(layered_costmap_->getCostmap()->getSizeInCellsX());
    auto out_height = static_cast<int>(layered_costmap_->getCostmap()->getSizeInCellsY());
    int mx, my;
    worldToMapEnforceBounds(robot_x, robot_y, mx, my);
    rolling_min_x_ = std::max(0, mx - out_width / 2);
    rolling_max_x_ = std::min(static_cast<int>(size_x_ - 1), mx + out_width / 2);
    rolling_min_y_ = std::max(0, my - out_height / 2);
    rolling_max_y_ = std::min(static_cast<int>(size_y_ - 1), my + out_height / 2);
  }
}

void KeepOutLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  if (!map_received_ || !enabled_) return;

  updating_ = true;
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  if (!rolling_window_) {
    rolling_min_x_ = min_i;
    rolling_max_x_ = max_i;
    rolling_min_y_ = min_j;
    rolling_max_y_ = max_j;
  }

  for (int j = rolling_min_y_; j < rolling_max_y_; j++) {
    for (int i = rolling_min_x_; i < rolling_max_x_; i++) {
      double wx, wy;
      unsigned int mx, my;
      mapToWorld(i, j, wx, wy);
      if (!master_grid.worldToMap(wx, wy, mx, my)) continue;

      auto it = j * size_x_ + i;
      auto master_it = my * span + mx;
      auto grid_value = costmap_[it];
      if (grid_value == NO_INFORMATION) continue;

      auto old_grid_value = master_array[master_it];
      if (old_grid_value == NO_INFORMATION || old_grid_value < grid_value) {
        master_array[master_it] = grid_value;
      }
    }
  }

  updating_ = false;
}

void KeepOutLayer::reset() {
  if (map_received_) {
    resetMaps();
    setAllZonesCost();
  }
}

void KeepOutLayer::matchSize() {
  if (!rolling_window_) {
    Costmap2D* master = layered_costmap_->getCostmap();
    if (master->getSizeInCellsX() != size_x_ || master->getSizeInCellsY() != size_y_) {
      ROS_ERROR("[虚拟墙层] matchSize !rolling_window_ 尺寸不同，需要等待新地图。");
      map_received_ = false;
    }
  }
}

inline bool KeepOutLayer::findAvailableId(uint32_t& id, ZoneConstIter& iter) const {
  if (keep_out_zones_.empty()) {
    id = 0;
    return true;
  }

  id = keep_out_zones_.back().id + 1;
  iter = id_search_start_iter_;
  if (0 == id) {
    iter = keep_out_zones_.begin();
  } else if (iter == keep_out_zones_.end()) {
    return true;
  } else {
    id = (*iter).id + 1;
    ++iter;
  }

  while (iter != keep_out_zones_.end()) {
    if (id < (*iter).id) {
      return true;
    }

    id = (*iter).id + 1;
    ++iter;
  }

  return (0 != id);
}

bool KeepOutLayer::addZone(const KeepOutLayer::PointVector& edges, uint32_t& id, uint8_t cost) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  bool result = true;
  if (keep_out_zones_.empty()) {
    keep_out_zones_.emplace_back(0, cost, edges);
    id_search_start_iter_ = keep_out_zones_.end();
    id = 0;
  } else {
    ZoneConstIter iter;
    result = findAvailableId(id, iter);
    if (result) {
      ZoneIter inserted_iter = keep_out_zones_.emplace(iter, id, cost, edges);
      if (iter != keep_out_zones_.end()) {
        id_search_start_iter_ = inserted_iter;
      } else {
        id_search_start_iter_ = keep_out_zones_.end();
      }
    }
  }

  if (map_received_) {
    setZoneValue(costmap_, edges, cost, fill_zones_);
    for (auto const& e : edges) {
      addExtraBounds(e.x, e.y, e.x, e.y);
    }
  }
  return result;
}

void KeepOutLayer::setZoneValue(unsigned char* grid, const PointVector& zone, uint8_t value, bool fill_zone) {
  std::vector<PointInt> map_zone(zone.size());
  PointInt loc{0, 0};
  for (unsigned int i = 0; i < zone.size(); ++i) {
    worldToMapNoBounds(zone[i].x, zone[i].y, loc.x, loc.y);
    map_zone[i] = loc;
  }

  std::vector<PointInt> zone_cells;

  // get the cells that fill the zone
  rasterizeZone(map_zone, zone_cells, fill_zone);

  // set the cost of those cells
  for (auto const& p : zone_cells) {
    // check if point is outside bounds
    if (p.x < 0 || p.x >= size_x_) continue;
    if (p.y < 0 || p.y >= size_y_) continue;

    auto index = p.x + size_x_ * p.y;
    auto old_grid_value = grid[index];
    if (old_grid_value == NO_INFORMATION || old_grid_value < value) {
      grid[index] = value;
    }
  }
}

void KeepOutLayer::zoneOutlineCells(const std::vector<PointInt>& zone,
                                    std::vector<PointInt>& zone_cells) {
  for (unsigned int i = 0; i < zone.size() - 1; ++i) {
    raytrace(zone[i].x, zone[i].y, zone[i + 1].x, zone[i + 1].y, zone_cells);
  }
  if (!zone.empty()) {
    auto last_index = static_cast<unsigned int>(zone.size() - 1);
    raytrace(zone[last_index].x, zone[last_index].y, zone[0].x, zone[0].y, zone_cells);
  }
}

void KeepOutLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  PointInt pt{x0, y0};
  int n = 1 + dx + dy;
  int x_inc = (x1 > x0) ? 1 : -1;
  int y_inc = (y1 > y0) ? 1 : -1;
  int error = dx - dy;
  dx *= 2;
  dy *= 2;

  for (; n > 0; --n) {
    cells.emplace_back(pt);

    if (error > 0) {
      pt.x += x_inc;
      error -= dy;
    } else {
      pt.y += y_inc;
      error += dx;
    }
  }
}

void KeepOutLayer::rasterizeZone(const std::vector<PointInt>& zone,
                                 std::vector<PointInt>& zone_cells, bool fill) {
  // we need a minimum zone of a traingle
  if (zone.size() < 3) return;

  // first get the cells that make up the outline of the zone
  zoneOutlineCells(zone, zone_cells);

  if (!fill) return;

  int max_x = zone.front().x;
  int max_y = zone.front().y;
  int min_x = zone.front().x;
  int min_y = zone.front().y;
  for (int i = 1; i < zone.size(); i++) {
    if (zone.at(i).x > max_x) max_x = zone.at(i).x;
    if (zone.at(i).y > max_y) max_y = zone.at(i).y;
    if (zone.at(i).x < min_x) min_x = zone.at(i).x;
    if (zone.at(i).y < min_y) min_y = zone.at(i).y;
  }

  PointInt pt{};
  for (int i = min_x + 1; i < max_x; i++) {
    for (int j = min_y + 1; j < max_y; j++) {
      pt.x = i;
      pt.y = j;
      if (inZone(zone, pt)) {
        zone_cells.emplace_back(pt);
      }
    }
  }
}

bool KeepOutLayer::keepOutZoneSrv(costmap_plugins::keepOutZone::Request& req,
                                  costmap_plugins::keepOutZone::Response& res) {
  ROS_INFO("[虚拟墙层] 虚拟墙服务被调用");
  const auto add = [&](bool use_record) {
    if (use_record) req.zone = record_zone_;
    if (req.cost == 0) req.cost = LETHAL_OBSTACLE;
    auto size = req.zone.size();
    if (size < 2) return false;

    PointVector points;
    points.reserve(4);

    if (size == 2) {
      geometry_msgs::Point point_A;
      geometry_msgs::Point point_B;
      point_A.x = req.zone[0].point.x;
      point_A.y = req.zone[0].point.y;
      point_A.z = 0.0;
      points.emplace_back(point_A);
      point_B.x = req.zone[1].point.x;
      point_B.y = req.zone[1].point.y;
      point_B.z = 0.0;
      points.emplace_back(point_B);

      // calculate the normal vector for AB
      geometry_msgs::Point point_N;
      point_N.x = point_B.y - point_A.y;
      point_N.y = point_A.x - point_B.x;

      // get the absolute value of N to normalize and get
      // it to the length of the costmap resolution
      double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
      point_N.x = point_N.x / abs_N * resolution_;
      point_N.y = point_N.y / abs_N * resolution_;

      // calculate the new points to get a zone which can be filled
      geometry_msgs::Point point;
      point.x = point_A.x + point_N.x;
      point.y = point_A.y + point_N.y;
      points.emplace_back(point);

      point.x = point_B.x + point_N.x;
      point.y = point_B.y + point_N.y;
      points.emplace_back(point);
    } else {
      geometry_msgs::Point p;
      for (unsigned int i = 0; i < req.zone.size(); i++) {
        p.x = req.zone[i].point.x;
        p.y = req.zone[i].point.y;
        p.z = 0.0;
        points.emplace_back(p);
      }
    }
    ROS_INFO("多边形包含 %zu 个点:", req.zone.size());
    ROS_INFO_STREAM("[虚拟墙层] name_ = " << name_);
    for (const auto& pt : req.zone) {
      ROS_INFO("Point: (%.2f, %.2f, %.2f)", pt.point.x, pt.point.y, pt.point.z);
    }
    return addZone(points, res.id, req.cost);
  };

  switch (req.command) {
    case 0: {
      ROS_INFO("[虚拟墙层] 添加区域!");
      return add(false);
    }
    case 1: {
      ROS_WARN("[虚拟墙层] 移除虚拟墙 %d!", res.id);
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (keep_out_zones_.empty()) {
        ROS_WARN("虚拟墙为空，未移除任何区域");
        return true;
      }
      auto iter = keep_out_zones_.begin();
      while (iter != keep_out_zones_.end()) {
        if (req.id == (*iter).id) {
          if (id_search_start_iter_ == keep_out_zones_.begin()) {
            keep_out_zones_.pop_front();
            id_search_start_iter_ = keep_out_zones_.begin();
            reset();
            return true;
          }
          // If the id_search_start_iter_ will be deleted, move it to the front
          if (id_search_start_iter_ == iter) {
            --id_search_start_iter_;
          }
          keep_out_zones_.erase(iter);
          reset();
          return true;
        }
        // In ascending list, id < (*iter).id means there is no matched id in the list
        if (req.id < (*iter).id) {
          ROS_WARN("在虚拟墙列表中找不到匹配的 id！");
          return true;
        }
        ++iter;
      }
      ROS_WARN("在虚拟墙列表中找不到匹配的 id!");
      return true;
    }
    case 2: {
      ROS_WARN("[虚拟墙层] 清除虚拟墙!");
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!keep_out_zones_.empty()) {
        keep_out_zones_.clear();
      }
      reset();

      // Clear the JSON file
      try {
        // Get package path
        std::string package_path = ros::package::getPath("costmap_plugins");
        
        // Create full file path
        std::string filepath = package_path + "/file/" + name_ + "_" + global_map_name + "_virtual_wall.json";
        ROS_INFO_STREAM("[虚拟墙层] filepath = " << filepath);
        // Write an empty JSON structure to the file
        Json::Value root;
        root["walls"] = Json::arrayValue;  // Set "walls" to an empty array
        
        std::ofstream file_out(filepath.c_str(), std::ios::out);
        if (file_out.is_open()) {
          file_out << root.toStyledString();
          file_out.close();
          ROS_INFO("[虚拟墙层] 成功清除 %s 中的虚拟墙数据", 
                   filepath.c_str());
        } else {
          ROS_ERROR("[虚拟墙层] 打开配置文件%s 失败: %s", 
                    filepath.c_str(), strerror(errno));
        }
      } catch (const std::exception& e) {
        ROS_ERROR("[虚拟墙层]  清除虚拟墙数据时出错 %s", e.what());
      }

      return true;
    }
    case 3: {
      ROS_INFO("[虚拟墙层] 开始记录!");
      record_zone_.clear();
      recording_ = true;
      return true;
    }
    case 4: {
      ROS_INFO("[虚拟墙层] 停止记录，添加虚拟墙!");
      auto result = add(true);
      
      // Save virtual wall data in JSON format
      if (result) {
        try {
          // Get package path
          std::string package_path = ros::package::getPath("costmap_plugins");
          
          // Create full directory path
          std::string file_dir = package_path + "/file";
          
          // Ensure directory exists
          struct stat info;
          if (stat(file_dir.c_str(), &info) != 0) {
            // Directory does not exist, create it
            mode_t mode = 0777;
            if (mkdir(file_dir.c_str(), mode) != 0) {
              ROS_ERROR("[虚拟墙层] 创建目录%s 失败: %s", 
                        file_dir.c_str(), strerror(errno));
              record_zone_.clear();
              recording_ = false;
              return result;
            }
          }
          
          // Create full file path
          std::string filepath = file_dir + "/" + name_ + "_" + global_map_name+"_virtual_wall.json";
          ROS_INFO_STREAM("[虚拟墙层] filepath = " << filepath);
          // keep_out_layer_virtual_wall.json
          // Read existing data
          Json::Value root;
          std::ifstream file_in(filepath.c_str(), std::ios::in);
          if (file_in.is_open()) {
            file_in >> root;
            file_in.close();
          }
          
          // Determine the next available ID
          uint32_t next_id = 0;
          for (const auto& wall : root["walls"]) {
            uint32_t current_id = wall["id"].asUInt();
            if (current_id >= next_id) {
              next_id = current_id + 1;
            }
          }
          
          // Create new wall data with ID
          Json::Value new_wall;
          new_wall["id"] = next_id;
          new_wall["num_points"] = static_cast<int>(record_zone_.size());
          for (const auto& point : record_zone_) {
            Json::Value json_point;
            json_point["x"] = point.point.x;
            json_point["y"] = point.point.y;
            json_point["z"] = point.point.z;
            new_wall["points"].append(json_point);
          }
          
          // Append new wall data to the array
          root["walls"].append(new_wall);
          
          // Write updated data to JSON file
          std::ofstream file_out(filepath.c_str(), std::ios::out);
          if (file_out.is_open()) {
            file_out << root.toStyledString();
            file_out.close();
            ROS_INFO("[虚拟墙层] 成功将虚拟墙数据保存到%s", 
                     filepath.c_str());
          } else {
            ROS_ERROR("[虚拟墙层] 打开文件 %s 写入失败: %s", 
                      filepath.c_str(), strerror(errno));
          }
        } catch (const std::exception& e) {
          ROS_ERROR("[虚拟墙层] 保存虚拟墙数据时出错: %s", e.what());
        }
      }
      
      record_zone_.clear();
      recording_ = false;
      return result;
    }
    default:
      return false;
  }
}

inline void KeepOutLayer::setAllZonesCost() {
  for (ZoneIter iter = keep_out_zones_.begin(); iter != keep_out_zones_.end(); ++iter) {
    setZoneValue(costmap_, (*iter).edges, (*iter).grid, fill_zones_);
  }
  addExtraBounds(origin_x_, origin_y_, origin_x_ + resolution_ * size_x_, origin_y_ + resolution_ * size_y_);
}

inline bool KeepOutLayer::inZone(const std::vector<PointInt>& zone, PointInt& point) {
  uint32_t size = zone.size();
  if (size < 3) {
    ROS_ERROR("[虚拟墙层] 获得的区域少于 3 个点");
    return false;
  }
  // Count how many time a ray start at (p_x, p_y) point to x dir intersects with the zone
  // Even->outside  Odd->inside
  // Robot pose
  double p_x = point.x;
  double p_y = point.y;
  // Counter and other variable
  int counter = 0;
  double xinters;
  // Initial zone point
  double p1_x, p1_y, p2_x, p2_y;
  p1_x = zone.back().x;
  p1_y = zone.back().y;

  for (int i = 0; i < size; i++) {
    p2_x = zone[i].x;
    p2_y = zone[i].y;
    if (p1_y == p2_y) {
      p1_x = p2_x;  // Update p1
      continue;
    }
    if (p_y > std::min(p1_y, p2_y) && p_y <= std::max(p1_y, p2_y) && p_x <= std::max(p1_x, p2_x)) {
      xinters = (p_y - p1_y) * (p2_x - p1_x) / (p2_y - p1_y) + p1_x;
      if (p1_x == p2_x || p_x <= xinters) {
        counter++;
      }
    }
    // update p1
    p1_x = p2_x;
    p1_y = p2_y;
  }
  return counter % 2 != 0;
}

}  // namespace costmap_2d
