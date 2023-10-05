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

#include <QDateTime>
#include <QDebug>
#include <QDialog>
#include <QFileDialog>
#include <QFileInfoList>
#include <QFileInfo>
#include <QProcess>
#include <QProgressBar>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QStringList>
#include <QTimer>

#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

struct FabricData
{
  std::string topic;
  std::string subscriber_node;
  std::string publisher_node;
  long long ros_layer_transmission_time;
  int ros_layer_number_of_dropped_messages;
  int ros_layer_accumulative_receive_rate;
  double frequency;
  std::string bandwidth;
  long long rmw_layer_transmission_time;
};

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
  QString workspace_path;
  std::shared_ptr < QProcess > process_launch;
  std::shared_ptr < QTimer > timer_launch;
  std::unordered_map < std::string, std::vector < FabricData >> fabric_data_map;
  void get_log_process();
  void load_latest_csv();
  void plot_raw_data_table(const QString latestCSVFilePath);

signals:
  void closeRequested();

protected:
  void closeEvent(QCloseEvent * event) override;

private slots:
  void on_pushButtonWorkspacePath_clicked();
  void on_pushButtonConfigPath_clicked();
  void on_pushButtonLaunch_clicked();
  void on_pushButtonLaunchPause_clicked();
};

#endif // FABRIC_GUI_H
