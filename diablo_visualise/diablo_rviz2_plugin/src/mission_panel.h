#ifndef MISSION_PANEL_H
#define MISSION_PANEL_H


#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <QLabel>
#include <QTimer>
#include <QPixmap>
#include <QPainter>
#include <QLineEdit>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "switch.h"
#include "teleop_button.h"

class QLineEdit;
namespace diablo_rviz2_control_plugin
{

class MissionPanel: public rviz_common::Panel
{
Q_OBJECT
public:
  MissionPanel( QWidget* parent = 0 );

  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;
  
protected Q_SLOTS:
  void trigger_service(bool msg, std::string service_name);
  void set_robot_status(bool msg);
  void set_mode(int mode_id);
  float map(float x, float in_min, float in_max, float out_min, float out_max);
  void set_height(int height);
  void set_order_id(int order_id);

protected:
  bool event(QEvent *event);

private:
  std::shared_ptr<DummyNode> dummy_node_;

  rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr mission_pub_;
  motion_msgs::msg::MotionCtrl missionMsg;

  QString icon_on_path_;
  QString icon_off_path_;
  TeleopButton* teleop_button_;

  SwitchButton* robot_switch_button_;
  
  QPushButton* get_down2_button_;
  QPushButton* stand_up_button_;
  QPushButton* get_down_button_;
  QLabel* label_;
  QLabel* height_label_;
  QSlider* height_slider_;

};

} //namespace diablo_rviz2_control_plugin

#endif // MISSION_PANEL_H