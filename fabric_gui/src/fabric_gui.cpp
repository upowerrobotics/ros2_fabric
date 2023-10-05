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

#include "fabric_gui/fabric_gui.h"
#include "ui_fabric_gui.h"

FabricGUI::FabricGUI(QWidget * parent)
: QDialog(parent),
  ui(new Ui::FabricGUI),
  process_launch(std::make_shared<QProcess>(this))
{
  ui->setupUi(this);

  ui->progressBarLaunch->setRange(0, 100);
}

FabricGUI::~FabricGUI()
{
  delete ui;
}

void FabricGUI::closeEvent(QCloseEvent * event)
{
  emit closeRequested();
  QDialog::closeEvent(event);
}

void FabricGUI::on_pushButtonWorkspacePath_clicked()
{
  workspace_path = QFileDialog::getExistingDirectory(
    this,                       // Parent
    tr("Select Workspace Folder"), // Dialog title
    "",                         // Start directory (empty means current directory or last visited)
    QFileDialog::ShowDirsOnly   // Show only directories, not files
  );

  if (!workspace_path.isEmpty()) {
    // Use the selected folder path as needed
    qDebug() << "Selected folder:" << workspace_path;

    ui->lineEditWorkspacePath->setText(workspace_path);
  }

  ui->pushButtonLaunch->setEnabled(true);
}

void FabricGUI::on_pushButtonConfigPath_clicked()
{
  config_path = QFileDialog::getOpenFileName(
    this,                   // Parent
    tr("Open Config"),      // Dialog title
    "",                     // Start directory (empty means current directory or last visited)
    tr("Config Files (*.yaml);;All Files (*)")      // File filter
  );

  if (!config_path.isEmpty()) {
    // Use the selected file path as needed
    qDebug() << "Selected file:" << config_path;

    ui->lineEditConfigPath->setText(config_path);
  }
}

void FabricGUI::on_pushButtonLaunch_clicked()
{
  // stop and delete the prev timer and disconnected
  if (timer_launch && timer_launch->isActive()) {
    disconnect(timer_launch.get(), nullptr, nullptr, nullptr);
    timer_launch->deleteLater();
  }

  // create new timer
  timer_launch = std::make_unique<QTimer>(this);

  // UI controls
  ui->pushButtonLaunch->setEnabled(false);
  ui->pushButtonLaunchPause->setEnabled(true);
  ui->progressBarLaunch->setValue(0);
  launch_progress = 0;

  // Switch RMW
  QString selectedDDS = ui->comboBoxDDS->currentText();
  if (selectedDDS == "CycloneDDS") {
    setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp", 1);
  } else if (selectedDDS == "FastRTPS") {
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
  } else if (selectedDDS == "eCal") {
    setenv("RMW_IMPLEMENTATION", "rmw_ecal_proto_cpp", 1);
  }

  QString program = "ros2";
  QStringList arguments;
  arguments << "launch" << "fabric_nodes" << "fabric_nodes.launch.py";
  if (!config_path.isEmpty()) {
    qDebug() << "Read config file from: " << config_path;
    arguments << "config-path:=" + config_path;
  }
  process_launch->start(program, arguments);

  connect(
    timer_launch.get(), &QTimer::timeout, [ = ]() mutable {
      launch_progress += 1;
      ui->progressBarLaunch->setValue(launch_progress);

      if (launch_progress >= 100) {
        if (process_launch->state() == QProcess::Running) {
          process_launch->terminate();
          process_launch->waitForFinished(5000);
        }
        qDebug() << "launch finished";
        get_log_process();
        load_latest_csv();
        timer_launch->stop();
      }
    }
  );

  timer_launch->start(650);

  connect(
    process_launch.get(), QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    [ = ](int exitCode, QProcess::ExitStatus exitStatus) {
      ui->pushButtonLaunch->setEnabled(true);
      ui->pushButtonLaunchPause->setEnabled(false);
    });
}

void FabricGUI::on_pushButtonLaunchPause_clicked()
{
  if (process_launch->state() == QProcess::Running) {
    process_launch->terminate();
    process_launch->waitForFinished(5000);
  }

  if (timer_launch && timer_launch->isActive()) {
    disconnect(timer_launch.get(), nullptr, nullptr, nullptr);
    timer_launch->stop();
  }

  qDebug() << "launch finished";
  get_log_process();
  load_latest_csv();
  ui->pushButtonLaunch->setEnabled(true);
  ui->pushButtonLaunchPause->setEnabled(false);
}

void FabricGUI::get_log_process()
{
  std::unique_ptr<QProcess> getLogProcess;
  getLogProcess = std::make_unique<QProcess>();
  QString program = "ros2";
  QStringList arguments;
  arguments << "run" << "fabric_nodes" << "get_log.py";
  getLogProcess->start(program, arguments);
  if (getLogProcess->waitForFinished(-1)) {
    qDebug() << "get_log.py process finished";
  } else {
    qDebug() << "Error: get_log.py process did not finish in a reasonable time.";
  }
}

void FabricGUI::load_latest_csv()
{
  QDir dir(workspace_path);
  QStringList filters;
  filters << "*.csv";
  QFileInfoList fileInfoList = dir.entryInfoList(filters, QDir::Files);

  if (fileInfoList.isEmpty()) {
    qDebug() << "No CSV files found in the folder.";
    return;
  }

  QDateTime latestTimestamp;
  QString latestCSVFilePath;
  foreach(const QFileInfo & fileInfo, fileInfoList) {
    QDateTime fileTimestamp = fileInfo.lastModified();

    if (fileTimestamp > latestTimestamp) {
      latestTimestamp = fileTimestamp;
      latestCSVFilePath = fileInfo.absoluteFilePath();
    }
  }

  if (latestCSVFilePath.isEmpty()) {
    qDebug() << "No CSV files found in the folder.";
    return;
  }

  qDebug() << "Latest CSV file:" << latestCSVFilePath;

  plot_raw_data_table(latestCSVFilePath);
}

void FabricGUI::plot_raw_data_table(const QString latestCSVFilePath)
{
  std::ifstream file(latestCSVFilePath.toStdString());
  if (!file.is_open()) {
    qDebug() << "Can't open csv:" << latestCSVFilePath;
    return;
  }

  QStandardItemModel * model = new QStandardItemModel(this);
  model->setHorizontalHeaderLabels(
    QStringList() << "Topic" << "Sub Node" << "Pub Node" << "ROS Layer XMT"
                  << "Frequency" << "Bandwidth" << "RMW Layer XMT");
  ui->tableViewMeasurementResult->setModel(model);

  std::string line;
  bool skipFirstLine = true;
  while (std::getline(file, line)) {
    if (skipFirstLine) {
      skipFirstLine = false;
      continue;
    }

    std::istringstream iss(line);
    FabricData data;

    if (std::getline(iss, data.topic, '\t') &&
      std::getline(iss, data.subscriber_node, '\t') &&
      std::getline(iss, data.publisher_node, '\t'))
    {

      std::string temp;
      if (std::getline(iss, temp, '\t')) {
        data.ros_layer_transmission_time = std::stoll(temp);
      }

      // Skip ROS Layer Subscriber Time
      std::getline(iss, temp, '\t');

      // Skip ROS Layer Publisher Time
      std::getline(iss, temp, '\t');

      if (std::getline(iss, temp, '\t')) {
        data.ros_layer_number_of_dropped_messages = std::stoi(temp);
      }

      if (std::getline(iss, temp, '\t')) {
        data.ros_layer_accumulative_receive_rate = std::stod(temp);
      }

      // Skip Timestamp
      std::getline(iss, temp, '\t');

      if (std::getline(iss, temp, '\t')) {
        data.frequency = std::stod(temp);
      }

      std::getline(iss, data.bandwidth, '\t');

      if (std::getline(iss, temp, '\t')) {
        data.rmw_layer_transmission_time = std::stoll(temp);
      }

      // Skip RMW Layer Subscriber Time
      std::getline(iss, temp, '\t');

      // Skip RMW Layer Publisher Time
      std::getline(iss, temp, '\t');

      fabric_data_map[data.topic].push_back(data);

      QList<QStandardItem *> row;
      row << new QStandardItem(QString::fromStdString(data.topic))
          << new QStandardItem(QString::fromStdString(data.subscriber_node))
          << new QStandardItem(QString::fromStdString(data.publisher_node))
          << new QStandardItem(QString::number(data.ros_layer_transmission_time))
          << new QStandardItem(QString::number(data.frequency))
          << new QStandardItem(QString::fromStdString(data.bandwidth))
          << new QStandardItem(QString::number(data.rmw_layer_transmission_time));

      model->appendRow(row);
    }
  }
  qDebug() << "Number of rows: " << model->rowCount();
  ui->tableViewMeasurementResult->resizeColumnsToContents();
  ui->tableViewMeasurementResult->update();
}
