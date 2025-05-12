# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './shenyeming.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(519, 304)
        self.gridLayout_2 = QtWidgets.QGridLayout(Form)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setSpacing(20)
        self.gridLayout.setObjectName("gridLayout")
        self.pushBut_drawZone = QtWidgets.QPushButton(Form)
        self.pushBut_drawZone.setObjectName("pushBut_drawZone")
        self.gridLayout.addWidget(self.pushBut_drawZone, 0, 0, 1, 2)
        self.pushBut_UsePath = QtWidgets.QPushButton(Form)
        self.pushBut_UsePath.setObjectName("pushBut_UsePath")
        self.gridLayout.addWidget(self.pushBut_UsePath, 1, 0, 1, 2)
        self.textEdit = QtWidgets.QTextEdit(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.textEdit.sizePolicy().hasHeightForWidth())
        self.textEdit.setSizePolicy(sizePolicy)
        self.textEdit.setMinimumSize(QtCore.QSize(0, 20))
        self.textEdit.setMaximumSize(QtCore.QSize(16777215, 30))
        self.textEdit.setObjectName("textEdit")
        self.gridLayout.addWidget(self.textEdit, 1, 2, 1, 2)
        self.label = QtWidgets.QLabel(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setMinimumSize(QtCore.QSize(30, 0))
        self.label.setMaximumSize(QtCore.QSize(30, 16777215))
        self.label.setTextFormat(QtCore.Qt.AutoText)
        self.label.setScaledContents(False)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 3, 0, 1, 1)
        self.textEdit_Log = QtWidgets.QTextEdit(Form)
        self.textEdit_Log.setObjectName("textEdit_Log")
        self.gridLayout.addWidget(self.textEdit_Log, 3, 1, 1, 3)
        self.pushBut_creatZPath = QtWidgets.QPushButton(Form)
        self.pushBut_creatZPath.setObjectName("pushBut_creatZPath")
        self.gridLayout.addWidget(self.pushBut_creatZPath, 2, 2, 1, 1)
        self.pushBut_drawWall = QtWidgets.QPushButton(Form)
        self.pushBut_drawWall.setObjectName("pushBut_drawWall")
        self.gridLayout.addWidget(self.pushBut_drawWall, 0, 2, 1, 1)
        self.pushBut_useWall = QtWidgets.QPushButton(Form)
        self.pushBut_useWall.setObjectName("pushBut_useWall")
        self.gridLayout.addWidget(self.pushBut_useWall, 0, 3, 1, 1)
        self.pushBut_creatOPath = QtWidgets.QPushButton(Form)
        self.pushBut_creatOPath.setObjectName("pushBut_creatOPath")
        self.gridLayout.addWidget(self.pushBut_creatOPath, 2, 3, 1, 1)
        self.pushBut_creatTeachPath = QtWidgets.QPushButton(Form)
        self.pushBut_creatTeachPath.setObjectName("pushBut_creatTeachPath")
        self.gridLayout.addWidget(self.pushBut_creatTeachPath, 2, 0, 1, 2)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.pushBut_drawZone.setText(_translate("Form", "绘制路径"))
        self.pushBut_UsePath.setText(_translate("Form", "启用路径"))
        self.label.setText(_translate("Form", "调试显示"))
        self.pushBut_creatZPath.setText(_translate("Form", "生成之字形路径"))
        self.pushBut_drawWall.setText(_translate("Form", "画虚拟墙"))
        self.pushBut_useWall.setText(_translate("Form", "启用虚拟墙"))
        self.pushBut_creatOPath.setText(_translate("Form", "生成回字形路径"))
        self.pushBut_creatTeachPath.setText(_translate("Form", "生成示教路径"))
