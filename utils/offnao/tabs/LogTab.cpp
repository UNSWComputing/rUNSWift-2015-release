/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include "LogTab.hpp"
#include <QDebug>

using namespace std;

LogTab::LogTab(const QString &name, const QString &command) : QMainWindow(),
sName(name), process(this), uiTab() {
   uiTab.setupUi(this);
   QFont font("Monospace");
   font.setStyleHint(QFont::TypeWriter);
   uiTab.teLog->setFont(font);

   setCentralWidget(uiTab.teLog); // A flaw in designer/uic means I need this dirty hack

   connect(uiTab.cbFind, SIGNAL(activated(const QString &)),
           this, SLOT(find(const QString &)));
   connect(uiTab.cbFind, SIGNAL(editTextChanged(const QString &)),
           this, SLOT(unfind(const QString &)));

   connect(&process, SIGNAL(readyReadStandardOutput()),
           this, SLOT(readAllStandardOutput()));
   connect(&process, SIGNAL(readyReadStandardError()),
           this, SLOT(readAllStandardError()));
//   connect(&process, SIGNAL(stateChanged(QProcess::ProcessState)),
//           this, SLOT(stateChanged(QProcess::ProcessState)));
   connect(&process, SIGNAL(error(QProcess::ProcessError)),
           this, SLOT(error(QProcess::ProcessError)));
   connect(&process, SIGNAL(finished(int, QProcess::ExitStatus)),
           this, SLOT(finished(int, QProcess::ExitStatus)));
   process.start(command, QIODevice::ReadOnly | QIODevice::Text);
}

const QString &LogTab::name() const {
   return sName;
}

void LogTab::readAllStandardOutput() {
   uiTab.teLog->append(process.readAllStandardOutput());
}

void LogTab::readAllStandardError() {
   int fw = uiTab.teLog->fontWeight();
   uiTab.teLog->setFontWeight(QFont::Bold);
   uiTab.teLog->append(process.readAllStandardError());
   uiTab.teLog->setFontWeight(fw);
}

void LogTab::stateChanged(QProcess::ProcessState state) {
   qDebug() << __PRETTY_FUNCTION__ << " - " << state;
}

void LogTab::error(QProcess::ProcessError error) {
   qDebug() << __PRETTY_FUNCTION__ << " - " << error;
}

void LogTab::finished(int exitCode, QProcess::ExitStatus exitStatus) {
   qDebug() << __PRETTY_FUNCTION__ << " - " << exitCode << " - " << exitStatus;
   readAllStandardOutput();
   readAllStandardError();
}

void LogTab::find(const QString &text) {
   if (uiTab.teLog->find(text, QTextDocument::FindBackward))
      uiTab.cbFind->setStyleSheet("QComboBox { background-color: lightgreen; }");
   else
      uiTab.cbFind->setStyleSheet("QComboBox { background-color: red; }");
}

void LogTab::unfind(const QString &text) {
   uiTab.cbFind->setStyleSheet("QComboBox { background-color: white; }");
}

LogTab::~LogTab() {
   process.terminate();
   process.waitForFinished(1000);
}
