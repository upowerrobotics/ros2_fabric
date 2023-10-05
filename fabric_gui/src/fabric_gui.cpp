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
  progressbar_value = 0;

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
      progressbar_value += 1;
      ui->progressBarLaunch->setValue(progressbar_value);

      if (progressbar_value >= 100) {
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

  plot_rawdata_table(latestCSVFilePath);
  plot_average_table();
}

void FabricGUI::plot_rawdata_table(const QString latestCSVFilePath)
{
  std::ifstream file(latestCSVFilePath.toStdString());
  if (!file.is_open()) {
    qDebug() << "Can't open csv:" << latestCSVFilePath;
    return;
  }

  QStandardItemModel * model = new QStandardItemModel(this);
  model->setHorizontalHeaderLabels(
    QStringList() << "Topic" << "Sub Node" << "Pub Node" << "ROS Layer XMT"
                  << "RMW Layer XMT" << "Frequency" << "Bandwidth");
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
          << new QStandardItem(QString::number(data.rmw_layer_transmission_time))
          << new QStandardItem(QString::number(data.frequency))
          << new QStandardItem(QString::fromStdString(data.bandwidth));

      model->appendRow(row);
    }
  }
  qDebug() << "Number of rows: " << model->rowCount();
  ui->tableViewMeasurementResult->resizeColumnsToContents();
  ui->tableViewMeasurementResult->update();
}

void FabricGUI::plot_average_table()
{
  for (const auto & entry : fabric_data_map) {
    const std::string & topic = entry.first;
    const std::vector<FabricData> & data_vector = entry.second;

    FabricData average_data;
    average_data.topic = topic;
    average_data.subscriber_node = data_vector[0].subscriber_node;
    average_data.publisher_node = data_vector[0].publisher_node;
    average_data.ros_layer_transmission_time = 0;
    average_data.rmw_layer_transmission_time = 0;
    average_data.frequency = 0;
    double total_bandwidth = 0.0;
    int max_dropped_messages = std::numeric_limits<int>::min();
    int min_receive_rate = std::numeric_limits<int>::max();

    for (const FabricData & data : data_vector) {
      average_data.ros_layer_transmission_time += data.ros_layer_transmission_time;
      average_data.rmw_layer_transmission_time += data.rmw_layer_transmission_time;
      total_bandwidth += parse_bandwidth(data.bandwidth);
      average_data.frequency += data.frequency;
      max_dropped_messages = std::max(
        max_dropped_messages,
        data.ros_layer_number_of_dropped_messages);
      min_receive_rate = std::min(min_receive_rate, data.ros_layer_accumulative_receive_rate);
    }

    int count = data_vector.size();
    average_data.ros_layer_transmission_time /= count;
    average_data.rmw_layer_transmission_time /= count;
    average_data.frequency /= count;
    average_data.bandwidth = format_bandwidth(total_bandwidth / count);
    average_data.ros_layer_number_of_dropped_messages = max_dropped_messages;
    average_data.ros_layer_accumulative_receive_rate = min_receive_rate;

    fabric_average_map[topic] = average_data;
  }

  QStandardItemModel * model = new QStandardItemModel(this);
  model->setHorizontalHeaderLabels(
    QStringList() << "Topic" << "Sub Node" << "Pub Node" << "ROS Layer XMT"
                  << "RMW Layer XMT" << "Frequency" << "Bandwidth");
  ui->tableViewAverageResult->setModel(model);

  for (const auto & entry : fabric_average_map) {
    const FabricData & data = entry.second;

    QList<QStandardItem *> row;
    row << new QStandardItem(QString::fromStdString(data.topic))
        << new QStandardItem(QString::fromStdString(data.subscriber_node))
        << new QStandardItem(QString::fromStdString(data.publisher_node))
        << new QStandardItem(QString::number(data.ros_layer_transmission_time))
        << new QStandardItem(QString::number(data.rmw_layer_transmission_time))
        << new QStandardItem(QString::number(data.frequency))
        << new QStandardItem(QString::fromStdString(data.bandwidth));

    model->appendRow(row);
  }

  ui->tableViewAverageResult->resizeColumnsToContents();
  ui->tableViewAverageResult->update();
}

double FabricGUI::parse_bandwidth(const std::string & bandwidth_str)
{
  double multiplier = 1.0;
  if (bandwidth_str.find("KB") != std::string::npos) {
    multiplier = 1024.0;
  } else if (bandwidth_str.find("MB") != std::string::npos) {
    multiplier = 1024.0 * 1024.0;
  } else if (bandwidth_str.find("GB") != std::string::npos) {
    multiplier = 1024.0 * 1024.0 * 1024.0;
  }

  double bandwidth_value;
  std::istringstream(bandwidth_str) >> bandwidth_value;

  return bandwidth_value * multiplier;
}

std::string FabricGUI::format_bandwidth(double bandwidth_value)
{
  std::ostringstream formatted_bandwidth_stream;

  if (bandwidth_value >= 1024.0 * 1024.0 * 1024.0) {
    formatted_bandwidth_stream << std::fixed << std::setprecision(2) <<
      bandwidth_value / (1024.0 * 1024.0 * 1024.0) << "GB";
  } else if (bandwidth_value >= 1024.0 * 1024.0) {
    formatted_bandwidth_stream << std::fixed << std::setprecision(2) <<
      bandwidth_value / (1024.0 * 1024.0) << "MB";
  } else if (bandwidth_value >= 1024.0) {
    formatted_bandwidth_stream << std::fixed << std::setprecision(2) << bandwidth_value / 1024.0 <<
      "KB";
  } else {
    formatted_bandwidth_stream << std::fixed << std::setprecision(2) << bandwidth_value << "B";
  }

  return formatted_bandwidth_stream.str();
}
