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

#include "teleop_button.h"

#include "unistd.h"

namespace diablo_rviz2_control_plugin
{
TeleopButton::TeleopButton(QWidget *)
: QGridLayout(),
  v_label_("Linear(m/s)"),
  w_label_("Angular(rad/s)"),
  bt_q_("◢"),
  bt_w_("↑"),
  bt_e_("◣"),
  bt_a_("↶"),
  bt_s_("↓"),
  bt_d_("↷"),
  bt_z_("u"),
  bt_x_("Restore"),
  bt_c_("n"),
  discover_topic_("Detect"),
  target_linear_velocity_(1.0),
  target_angular_velocity_(1.0)

{
  dummy_node_ = std::make_shared<DummyNode>("teleop_rviz_node");

  this->addWidget(&discover_topic_, 0, 0);
  this->addWidget(&cmd_topic_box_, 0, 1);

  linear_speed_box_.setRange(0, 2.0);
  linear_speed_box_.setValue(target_linear_velocity_);
  linear_speed_box_.setSingleStep(0.1);
  angular_speed_box_.setRange(0, 5.0);
  angular_speed_box_.setValue(target_angular_velocity_);
  angular_speed_box_.setSingleStep(0.1);
  QHBoxLayout * l1 = new QHBoxLayout;
  l1->addWidget(&v_label_);
  l1->addWidget(&linear_speed_box_);
  l1->addWidget(&w_label_);
  l1->addWidget(&angular_speed_box_);
  this->addLayout(l1, 1, 0, 1, -1);

  this->addWidget(&bt_q_, 2, 0, 1, 1);
  this->addWidget(&bt_w_, 2, 1, 1, 1);
  this->addWidget(&bt_e_, 2, 2, 1, 1);

  this->addWidget(&bt_a_, 3, 0);
  this->addWidget(&bt_s_, 3, 1);
  this->addWidget(&bt_d_, 3, 2);

  this->addWidget(&bt_z_, 4, 0);
  bt_x_.setStyleSheet("QPushButton {color: red;}");
  this->addWidget(&bt_x_, 4, 1);
  this->addWidget(&bt_c_, 4, 2);

  QTimer * output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(send_vel()));

  connect(&discover_topic_, &QPushButton::clicked, [this](bool) { discover_topics(); });
  connect(&cmd_topic_box_, SIGNAL(currentIndexChanged(int)), SLOT(update_topic(int)));
  connect(&linear_speed_box_, SIGNAL(valueChanged(double)), this, SLOT(set_linear_speed(double)));
  connect(&angular_speed_box_, SIGNAL(valueChanged(double)), this, SLOT(set_angular_speed(double)));

  connect(&bt_q_, &QPushButton::pressed, [this](void) { set_vel('q'); });
  connect(&bt_w_, &QPushButton::pressed, [this](void) { set_vel('w'); });
  connect(&bt_e_, &QPushButton::pressed, [this](void) { set_vel('e'); });
  connect(&bt_a_, &QPushButton::pressed, [this](void) { set_vel('a'); });
  connect(&bt_s_, &QPushButton::pressed, [this](void) { set_vel('s'); });
  connect(&bt_d_, &QPushButton::pressed, [this](void) { set_vel('d'); });
  connect(&bt_z_, &QPushButton::pressed, [this](void) { set_vel('z'); });
  connect(&bt_x_, &QPushButton::pressed, [this](void) { set_vel('x'); });
  connect(&bt_c_, &QPushButton::pressed, [this](void) { set_vel('c'); });

  connect(&bt_q_, &QPushButton::released, [this](void) { set_vel(' '); });
  connect(&bt_w_, &QPushButton::released, [this](void) { set_vel(' '); });
  connect(&bt_e_, &QPushButton::released, [this](void) { set_vel(' '); });
  connect(&bt_a_, &QPushButton::released, [this](void) { set_vel(' '); });
  connect(&bt_s_, &QPushButton::released, [this](void) { set_vel(' '); });
  connect(&bt_d_, &QPushButton::released, [this](void) { set_vel(' '); });
  connect(&bt_z_, &QPushButton::released, [this](void) { set_vel(' '); });
  connect(&bt_x_, &QPushButton::released, [this](void) { set_vel(' '); });
  connect(&bt_c_, &QPushButton::released, [this](void) { set_vel(' '); });

  output_timer->start(100);
}

void TeleopButton::key_to_button(QEvent * event)
{
  if (event->type() == QEvent::KeyPress) {
    QKeyEvent * k = static_cast<QKeyEvent *>(event);
    switch (k->key()) {
      case (Qt::Key_Q):
        set_vel('q');
        bt_q_.setDown(true);
        break;
      case (Qt::Key_W):
        set_vel('w');
        bt_w_.setDown(true);
        break;
      case (Qt::Key_E):
        set_vel('e');
        bt_e_.setDown(true);
        break;
      case (Qt::Key_A):
        set_vel('a');
        bt_a_.setDown(true);
        break;
      case (Qt::Key_S):
        set_vel('s');
        bt_s_.setDown(true);
        break;
      case (Qt::Key_D):
        set_vel('d');
        bt_d_.setDown(true);
        break;
      case (Qt::Key_Z):
        set_vel('z');
        bt_z_.setDown(true);
        break;
      case (Qt::Key_X):
        set_vel('x');
        bt_x_.setDown(true);
        break;
      case (Qt::Key_C):
        set_vel('c');
        bt_c_.setDown(true);
        break;
    }

  } else if (event->type() == QEvent::KeyRelease) {
    set_vel(' ');
    bt_q_.setDown(false);
    bt_w_.setDown(false);
    bt_e_.setDown(false);
    bt_a_.setDown(false);
    bt_s_.setDown(false);
    bt_d_.setDown(false);
    bt_z_.setDown(false);
    bt_x_.setDown(false);
    bt_c_.setDown(false);
  }
}

void TeleopButton::set_linear_speed(double value) { target_linear_velocity_ = value; }

void TeleopButton::set_angular_speed(double value) { target_angular_velocity_ = value; }

void TeleopButton::set_vel(const char & key)
{
  motion_msgs::msg::MotionCtrl missionMsg;
  switch (key) {
    case 'w':
      target_direction_ = CMD_GO_FORWARD;
      target_speed_ = target_linear_velocity_;
      missionMsg.value.forward=target_linear_velocity_;
      break;
    case 'a':
      target_direction_ = CMD_GO_LEFT;
      target_speed_ = target_angular_velocity_;
      missionMsg.value.left=target_angular_velocity_;
      break;
    case 's':
      target_direction_ = CMD_GO_FORWARD;
      target_speed_ = -1.0 * target_linear_velocity_;
      missionMsg.value.forward=-1.0*target_linear_velocity_;
      break;
    case 'd':
      target_direction_ = CMD_GO_LEFT;
      target_speed_ = -1.0 * target_angular_velocity_;
      missionMsg.value.left=-1.0*target_angular_velocity_;
      break;
    case 'q':
      target_direction_ = CMD_ROLL_RIGHT;
      target_speed_ = -0.2;
      missionMsg.value.roll= -0.2;
      break;
    case 'e':
      target_direction_ = CMD_ROLL_RIGHT;
      target_speed_ = 0.2;
      missionMsg.value.roll= -0.2;
      break;
    case 'z':
      target_direction_ = CMD_PITCH;
      target_speed_ = 0.5;
      missionMsg.value.pitch = 0.5;
      break;
    case 'x':
      restore_pose();
      break;
    case 'c':
      target_direction_ = CMD_PITCH;
      target_speed_ = -0.5;
      missionMsg.value.pitch = -0.5;      
      break;
    case ' ':
      missionMsg.value.up = 1.0;
    default:
      break;
  }
  linear_velocity_ =
    target_linear_velocity_ * ((key == 'q') + (key == 'w') + (key == 'e') + -1 * (key == 'x'));
  lateral_velocity_ = target_linear_velocity_ * ((key == 'z') + -1 * (key == 'c'));

  angular_velocity_ = target_angular_velocity_ *
                      ((key == 'q') + (key == 'a') + -1 * (key == 'e') + -1 * (key == 'd'));
  if (cmd_topic_selected_) {
    missionMsg.value.up = 1.0;
    cmd_pub_->publish(missionMsg);
  }                  
}

void TeleopButton::restore_pose()
{
  motion_msgs::msg::MotionCtrl missionMsg;
  if (cmd_topic_selected_) {
    missionMsg.value.roll=0.0;
    cmd_pub_->publish(missionMsg);
    usleep(1000 * 100);
    missionMsg.value.pitch=0.0;
    cmd_pub_->publish(missionMsg);
  }
}
// void TeleopButton::send_vel()
// {
//   motion_msgs::msg::MotionCtrl missionMsg;
//   // // missionMsg.cmd_id = target_direction_;
//   // // missionMsg.value = target_speed_;

//   // if (cmd_topic_selected_) {
//   //   cmd_pub_->publish(missionMsg);
//   // }
// }

void TeleopButton::update_topic(int pos)
{
  if (pos >= 0) setTopic(cmd_topic_list_[pos]);
}

void TeleopButton::discover_topics()
{
  std::string allowed_topic = "motion_msgs/msg/MotionCtrl";

  cmd_topic_list_.clear();
  cmd_topic_list_.emplace_back(" ");

  auto topics_and_types = dummy_node_->get_topic_names_and_types();
  for (auto it : topics_and_types) {
    for (auto type : it.second) {
      if (type == allowed_topic) {
        cmd_topic_list_.emplace_back(QString::fromStdString(it.first));
      }
    }
  }

  cmd_topic_box_.clear();
  for (size_t pos = 0; pos < cmd_topic_list_.size(); pos++) {
    cmd_topic_box_.addItem(cmd_topic_list_[pos]);
  }
}

void TeleopButton::setTopic(const QString & topic)
{
  if (topic == "" || topic == " ") {
    cmd_topic_selected_ = false;
  } else {
    try {
      cmd_pub_ = dummy_node_->create_publisher<motion_msgs::msg::MotionCtrl>(
        topic.toStdString(), rclcpp::SystemDefaultsQoS());
      std::cout << "set velocity topic to: " << topic.toStdString() << std::endl;
      cmd_topic_selected_ = true;
    } catch (...) {
      std::cerr << "Unable to initialize velocity publisher, please verify topic name."
                << std::endl;
    }
  }
}

}  // namespace diablo_rviz2_control_plugin
