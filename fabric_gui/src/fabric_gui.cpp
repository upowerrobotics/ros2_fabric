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

#include <QDebug>
#include <QTimer>

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

void FabricGUI::on_pushButtonLaunch_clicked()
{
  ui->pushButtonLaunch->setEnabled(false);
  ui->progressBarLaunch->setValue(0);
  launch_progress = 0;

  // stop and delete the prev timer and disconnected
  if (timer_launch && timer_launch->isActive()) {
    timer_launch->stop();
    disconnect(timer_launch.get(), nullptr, nullptr, nullptr);
    timer_launch->deleteLater();
  }

  // create new timer
  timer_launch = std::make_unique<QTimer>(this);

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
        timer_launch->stop();
        if (process_launch->state() == QProcess::Running) {
          process_launch->terminate();
          process_launch->waitForFinished(5000);
        }
        timer_launch->deleteLater();
        process_launch->deleteLater();

        ui->pushButtonLaunch->setEnabled(true);
      }
    }
  );

  timer_launch->start(700);

  connect(
    process_launch.get(), QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
      [ = ](int exitCode, QProcess::ExitStatus exitStatus) {
      qDebug() << "launch finished";
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
