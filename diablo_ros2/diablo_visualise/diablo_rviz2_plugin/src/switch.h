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

#ifndef SWITCH_H_
#define SWITCH_H_

#include <QtWidgets>
#include <string>

class SwitchButton : public QWidget
{
  Q_OBJECT
  Q_DISABLE_COPY(SwitchButton)

public:
  enum Style { YESNO, ONOFF, BOOL, EMPTY };

public:
  explicit SwitchButton(QWidget * parent = nullptr, Style style = Style::ONOFF);
  ~SwitchButton() override;

  //-- QWidget methods
  void mousePressEvent(QMouseEvent *) override;
  void paintEvent(QPaintEvent * event) override;
  void setEnabled(bool);

  //-- Setters
  void setDuration(int);
  void setValue(bool);
  void setServiceName(std::string);

  //-- Getters
  bool value() const;

Q_SIGNALS:
  void valueChanged(bool newvalue, std::string msg = "");

private:
  class SwitchCircle;
  class SwitchBackground;
  void _update();

private:
  bool _value;
  int _duration;
  std::string _service_name;

  QLinearGradient _lg;
  QLinearGradient _lg2;
  QLinearGradient _lg_disabled;

  QColor _pencolor;
  QColor _offcolor;
  QColor _oncolor;
  int _tol;
  int _borderradius;

  // This order for definition is important (these widgets overlap)
  QLabel * _labeloff;
  SwitchBackground * _background;
  QLabel * _labelon;
  SwitchCircle * _circle;

  bool _enabled;

  QPropertyAnimation * __btn_move;
  QPropertyAnimation * __back_move;
};

class SwitchButton::SwitchBackground : public QWidget
{
  Q_OBJECT
  Q_DISABLE_COPY(SwitchBackground)

public:
  explicit SwitchBackground(
    QWidget * parent = nullptr, QColor color = QColor(154, 205, 50), bool rect = false);
  ~SwitchBackground() override;

  //-- QWidget methods
  void paintEvent(QPaintEvent * event) override;
  void setEnabled(bool);

private:
  bool _rect;
  int _borderradius;
  QColor _color;
  QColor _pencolor;
  QLinearGradient _lg;
  QLinearGradient _lg_disabled;

  bool _enabled;
};

class SwitchButton::SwitchCircle : public QWidget
{
  Q_OBJECT
  Q_DISABLE_COPY(SwitchCircle)

public:
  explicit SwitchCircle(
    QWidget * parent = nullptr, QColor color = QColor(255, 255, 255), bool rect = false);
  ~SwitchCircle() override;

  //-- QWidget methods
  void paintEvent(QPaintEvent * event) override;
  void setEnabled(bool);

private:
  bool _rect;
  int _borderradius;
  QColor _color;
  QColor _pencolor;
  QRadialGradient _rg;
  QLinearGradient _lg;
  QLinearGradient _lg_disabled;

  bool _enabled;
};

#endif  // SWITCH_H_
