// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MISSION_PANEL_H_
#define MISSION_PANEL_H_

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPixmap>
#include <QTimer>
#include <QVBoxLayout>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/panel.hpp"
#include "switch.h"
#include "teleop_button.h"

class QLineEdit;
namespace diablo_rviz2_control_plugin
{

class MissionPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit MissionPanel(QWidget * parent = 0);

  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

protected Q_SLOTS:
  void trigger_service(bool msg, std::string service_name);
  void set_robot_status(bool msg);
  void set_mode(int mode_id);
  float map(float x, float in_min, float in_max, float out_min, float out_max);
  void set_height(int height);
  void set_order_id(int order_id);

protected:
  bool event(QEvent * event);

private:
  std::shared_ptr<DummyNode> dummy_node_;

  rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr mission_pub_;
  motion_msgs::msg::MotionCtrl missionMsg;

  QString icon_on_path_;
  QString icon_off_path_;
  TeleopButton * teleop_button_;

  SwitchButton * robot_switch_button_;

  QPushButton * get_down2_button_;
  QPushButton * stand_up_button_;
  QPushButton * get_down_button_;
  QLabel * label_;
  QLabel * height_label_;
  QSlider * height_slider_;
};

}  // namespace diablo_rviz2_control_plugin

#endif  // MISSION_PANEL_H_
