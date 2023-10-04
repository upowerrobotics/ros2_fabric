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
#include <QProcess>
#include <QTimer>

FabricGUI::FabricGUI(QWidget * parent)
: QDialog(parent),
  ui(new Ui::FabricGUI)
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
  ui->progressBarLaunch->setValue(0);

  QProcess * process = new QProcess(this);
  QString program = "ros2";
  QStringList arguments;
  arguments << "launch" << "fabric_nodes" << "fabric_nodes.launch.py";
  if (!config_path.isEmpty()) {
    qDebug() << "Read config file from: " << config_path;
    arguments << "config-path:=" + config_path;
  }
  process->start(program, arguments);

  QTimer * progressTimer = new QTimer(this);
  int progress = 0;

  connect(
    progressTimer, &QTimer::timeout, [ = ]() mutable {
      progress += 1;
      ui->progressBarLaunch->setValue(progress);

      if (progress >= 100) {
        progressTimer->stop();
        if (process->state() == QProcess::Running) {
          process->terminate();
          process->waitForFinished(5000);
        }
        progressTimer->deleteLater();
        process->deleteLater();
      }
    });

  progressTimer->start(700);

  connect(
    process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    [ = ](int exitCode, QProcess::ExitStatus exitStatus) {
      qDebug() << "launch finished";
    });
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
