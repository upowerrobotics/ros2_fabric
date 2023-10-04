// Copyright 2023 U Power Robotics USA, Inc.
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

#ifndef FABRIC_GUI__FABRIC_GUI_H
#define FABRIC_GUI_FABRIC_GUI_H

#include <QDialog>
#include <QFileDialog>
#include <QProcess>
#include <QProgressBar>

namespace Ui {
  class FabricGUI;
}

class FabricGUI: public QDialog {
  Q_OBJECT

public:
  explicit FabricGUI(QWidget * parent = nullptr);
  ~FabricGUI();

private:
  Ui::FabricGUI * ui;
  int launch_progress;
  QString config_path;
  std::shared_ptr<QProcess> process_launch;
  std::shared_ptr<QTimer> timer_launch;

signals:
  void closeRequested();

protected:
  void closeEvent(QCloseEvent * event) override;

private slots:
  void on_pushButtonLaunch_clicked();
  void on_pushButtonLaunchPause_clicked();
  void on_pushButtonConfigPath_clicked();  
};

#endif // FABRIC_GUI_H
