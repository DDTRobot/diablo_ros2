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

#include "mission_panel.h"

#include <memory>
#include "unistd.h"
namespace diablo_rviz2_control_plugin
{
MissionPanel::MissionPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  dummy_node_ = std::make_shared<DummyNode>("diablo_panel_node");
  setFocusPolicy(Qt::ClickFocus);
  mission_pub_ = dummy_node_->create_publisher<motion_msgs::msg::MotionCtrl>(
    "diablo/MotionCmd", rclcpp::SystemDefaultsQoS());
  icon_off_path_ = QString::fromStdString(
    ament_index_cpp::get_package_share_directory("diablo_rviz2_control_plugin") +
    "/data/ddt_off_64.png");
  icon_on_path_ = QString::fromStdString(
    ament_index_cpp::get_package_share_directory("diablo_rviz2_control_plugin") +
    "/data/ddt_on_64.png");

  QVBoxLayout * layout = new QVBoxLayout;
  QHBoxLayout * mode_box_layout = new QHBoxLayout;

  // Set header
  label_ = new QLabel;
  QPixmap pic(icon_off_path_);
  label_->setPixmap(pic);
  mode_box_layout->addWidget(label_);

  // Top SwitchButton
  robot_switch_button_ = new SwitchButton(this, SwitchButton::Style::EMPTY);
  robot_switch_button_->setValue(false);
  stand_up_button_ = new QPushButton("Stand Up");
  stand_up_button_->setEnabled(false);
  get_down_button_ = new QPushButton("Get Down");
  get_down_button_->setEnabled(false);

  mode_box_layout->addWidget(robot_switch_button_);
  mode_box_layout->addWidget(stand_up_button_);
  mode_box_layout->addWidget(get_down_button_);

  // teleop layout
  QGroupBox * teleop_group_box = new QGroupBox(tr("🕹  Teleop layout"));
  teleop_button_ = new TeleopButton(this);

  height_slider_ = new QSlider(Qt::Vertical);
  height_slider_->setMinimum(22);
  height_slider_->setMaximum(49);
  height_slider_->setValue(49);
  height_label_ = new QLabel(QString::number(49));
  QVBoxLayout * height_layout = new QVBoxLayout;
  height_layout->addWidget(height_label_);
  height_layout->addWidget(height_slider_);
  height_slider_->setEnabled(false);

  QHBoxLayout * teleop_layout = new QHBoxLayout;
  teleop_layout->addLayout(teleop_button_);
  teleop_layout->addLayout(height_layout);
  teleop_group_box->setLayout(teleop_layout);

  // main layout
  layout->addLayout(mode_box_layout);
  layout->addWidget(teleop_group_box);
  setLayout(layout);

  connect(robot_switch_button_, SIGNAL(valueChanged(bool)), this, SLOT(set_robot_status(bool)));
  connect(stand_up_button_, &QPushButton::clicked, [this](void) { set_mode(1); });
  connect(get_down_button_, &QPushButton::clicked, [this](void) { set_mode(0); });
  connect(height_slider_, SIGNAL(valueChanged(int)), SLOT(set_height(int)));
}

void MissionPanel::set_robot_status(bool msg)
{
  if (msg) {
    std::cout << "set_robot_status " << msg << std::endl;
    label_->setPixmap(QPixmap(icon_on_path_));
    stand_up_button_->setEnabled(true);
    get_down_button_->setEnabled(true);
    height_slider_->setEnabled(true);
  } else {
    std::cout << "set_robot_status " << msg << std::endl;
    label_->setPixmap(QPixmap(icon_off_path_));
    stand_up_button_->setEnabled(false);
    get_down_button_->setEnabled(false);
    height_slider_->setEnabled(false);
  }
}

float MissionPanel::map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MissionPanel::set_height(int height)
{
  // missionMsg.cmd_id = CMD_BODY_UP;
  missionMsg.value.up = map((float)height, 22.0, 49.0, 0.0, 1.0);
  // heightMsg.value = ((int)(map((float)height,22.0,49.0,0.0,1.0)*10))/(10.0);
  mission_pub_->publish(missionMsg);
  height_label_->setText(QString::number(height));
}

bool MissionPanel::event(QEvent * event)
{
  teleop_button_->key_to_button(event);
  return 0;
}

void MissionPanel::set_mode(int mode_id)
{
  missionMsg.value.up = 1.0;
  if (mode_id) {
    missionMsg.mode_mark=true;
    missionMsg.mode.stand_mode = true;
  } else {
    missionMsg.mode_mark=true;
    missionMsg.mode.stand_mode = false;
  }
  mission_pub_->publish(missionMsg);
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  missionMsg.mode_mark=false;
  mission_pub_->publish(missionMsg);
  // std::cout<<"Changed mode to "<< (mode_id>0? "MANUAL":"DEFAULT " )<< std::endl;
}

void MissionPanel::save(rviz_common::Config config) const { rviz_common::Panel::save(config); }

// Load all configuration data for this panel from the given Config object.
void MissionPanel::load(const rviz_common::Config & config) { rviz_common::Panel::load(config); }

}  // namespace diablo_rviz2_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(diablo_rviz2_control_plugin::MissionPanel, rviz_common::Panel)
