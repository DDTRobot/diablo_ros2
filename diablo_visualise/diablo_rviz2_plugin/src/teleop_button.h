
#ifndef TELEOP_WIDGET_H
#define TELEOP_WIDGET_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "motion_msgs/msg/motion_ctrl.hpp"

#include <QEvent>
#include <QLabel>
#include <QTimer>
#include <QWidget>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QKeyEvent>
#include <QHBoxLayout>
#include <QPushButton>
#include <QGridLayout>
#include <QDoubleSpinBox>

#define CMD_GO_FORWARD                               0x08
#define CMD_GO_LEFT                                  0x04
#define CMD_ROLL_RIGHT                               0x09

#define CMD_HEIGH_MODE                               0x01
#define CMD_BODY_UP                                  0x11

#define CMD_STAND_UP                                 0x02
#define CMD_STAND_DOWN                               0x12

#define CMD_PITCH                                    0x03
#define CMD_PITCH_MODE                               0x13

#define CMD_SPEED_MODE                               0x05


namespace diablo_rviz2_control_plugin
{

class DummyNode : public rclcpp::Node
{
public:
    DummyNode(std::string node_name):Node(node_name){};
    ~DummyNode(){};
};

class TeleopButton : public QGridLayout
{
Q_OBJECT

public:
    TeleopButton(QWidget *parent = nullptr);
    void key_to_button(QEvent *event);

public Q_SLOTS:
    void setTopic( const QString& topic );

private Q_SLOTS:
    void discover_topics();
    void update_topic(int pos);
    void send_vel();
    void set_linear_speed(double value);
    void set_angular_speed(double value);    
    void restore_pose();

private:
    QLabel output_topic_label_;

    QDoubleSpinBox linear_speed_box_;
    QDoubleSpinBox angular_speed_box_;
    QLabel v_label_;
    QLabel w_label_;

    QPushButton bt_q_, bt_w_, bt_e_;
    QPushButton bt_a_, bt_s_, bt_d_;
    QPushButton bt_z_, bt_x_, bt_c_;

    void set_vel(const char &key);

    QComboBox cmd_topic_box_;
    QPushButton discover_topic_;
    std::vector<QString> cmd_topic_list_;

    rclcpp::Publisher<motion_msgs::msg::MotionCtrl>::SharedPtr cmd_pub_;
    
    double linear_velocity_;
    double target_linear_velocity_;
    double lateral_velocity_;
    double angular_velocity_;
    double target_angular_velocity_;
    float target_speed_;
    int32_t target_direction_;
    bool cmd_topic_selected_ = false;
    std::shared_ptr<DummyNode> dummy_node_;
    
};

}
#endif // TELEOP_WIDGET_H