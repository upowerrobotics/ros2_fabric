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

FabricGUI::FabricGUI(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FabricGUI) {
    ui->setupUi(this);
}

FabricGUI::~FabricGUI() {
    delete ui;
}
