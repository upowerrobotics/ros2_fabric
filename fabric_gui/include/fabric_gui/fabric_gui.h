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
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

/**
 * @brief A struct to store data related to the Fabric functions.
 */
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

/**
 * @brief The main class for the Fabric GUI application.
 */
class FabricGUI: public QDialog {
  Q_OBJECT

public:
  /**
   * @brief Constructor for the FabricGUI class.
   *
   * @param parent The parent widget.
   */
  explicit FabricGUI(QWidget * parent = nullptr);

  /**
   * @brief Destructor for the FabricGUI class.
   */
  ~FabricGUI();

private:
  Ui::FabricGUI * ui;  ///< The UI instance.
  int progressbar_value;  ///< Progressbar value of the fabric_nodes.launch.py.
  QString config_path;  ///< Path to configuration files.
  QString workspace_path;  ///< Path to the workspace.

  std::shared_ptr < QProcess > process_launch;  ///< Pointer to the fabric_nodes.launch.py.
  std::shared_ptr < QTimer > timer_launch;  ///< Timer for the fabric_nodes.launch.py.
  std::unordered_map < std::string, std::vector < FabricData >> fabric_data_map;  ///< Map to store FABRIC data
  std::unordered_map < std::string, FabricData > fabric_average_map;  ///< Map to store average FABRIC data.

  /**
   * @brief Run get_log.py.
   */
  void get_log_process();

  /**
   * @brief Load the latest CSV file.
   */
  void load_latest_csv();

  /**
   * @brief Plot raw data table.
   *
   * @param latestCSVFilePath The path to the latest CSV file.
   */
  void plot_rawdata_table(const QString latestCSVFilePath);

  /**
   * @brief Plot average data table.
   */
  void plot_average_table();

  /**
   * @brief Parse bandwidth string (B, KB, MB, GB) to a double value.
   *
   * @param bandwidth_str The bandwidth string.
   * @return The parsed bandwidth value.
   */
  double parse_bandwidth(const std::string & bandwidth_str);

  /**
   * @brief Format bandwidth value as a string (B, KB, MB, GB).
   *
   * @param bandwidth_value The bandwidth value.
   * @return The formatted bandwidth string.
   */
  std::string format_bandwidth(double bandwidth_value);

signals:
  /**
   * @brief Signal emitted when the GUI is requested to be closed.
   */
  void closeRequested();

protected:
  /**
   * @brief Overridden close event handler.
   *
   * @param event The close event.
   */
  void closeEvent(QCloseEvent * event) override;

private slots:
  void on_pushButtonWorkspacePath_clicked();
  void on_pushButtonConfigPath_clicked();
  void on_pushButtonLaunch_clicked();
  void on_pushButtonLaunchPause_clicked();
};

#endif // FABRIC_GUI_H
