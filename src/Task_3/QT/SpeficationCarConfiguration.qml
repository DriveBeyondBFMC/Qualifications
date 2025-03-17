import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Controls 2.12
import QtQuick.Shapes 1.12
RowLayout{
    spacing: 50

    SpeficationCarModel{
        id: speficationCarModel
    }



    ColumnLayout {
        //anchors.horizontalCenter: parent.horizontalCenter
        spacing: 30
        Layout.alignment: Qt.AlignRight
        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            spacing: 20
            ColumnLayout{
                Image{

                    id: temperatureIcon
                    height: 65
                    fillMode: Image.PreserveAspectFit
                    source: "qrc:/ui/icon/temperature.png"
                    sourceSize: Qt.size(45, 45)
                    smooth: true
                    visible: false

                }
                ColorOverlay {
                    anchors.fill: temperatureIcon
                    source: temperatureIcon
                    color: "gray"
                 }
            }

            ColumnLayout {
                Layout.alignment: Qt.AlignRight
                Text {
                    id: temperatureShow
                    Layout.alignment: Qt.AlignLeft
                    text: "80 C"
                    font.pixelSize: 23
                    font.bold: true
                    color: "white"
                }
                Text {
                    Layout.alignment: Qt.AlignLeft
                    text: qsTr("Temperature")
                    font.pixelSize: 15
                    color: "white"

                }
            }
        }



        ColumnLayout {
            //anchors.horizontalCenter: parent.horizontalCenter
            Layout.alignment: Qt.AlignRight
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 20
                ColumnLayout{
                    Image{
                        id: fuelStationIcon
                        height: 65
                        fillMode: Image.PreserveAspectFit
                        source: "qrc:/ui/icon/fuel-station.png"
                        sourceSize: Qt.size(45, 45)
                        smooth: true
                        visible: false

                    }
                    ColorOverlay {
                        anchors.fill: fuelStationIcon
                        source: fuelStationIcon
                        color: "gray"
                     }
                }

                ColumnLayout {
                    Layout.alignment: Qt.AlignRight | Qt.AlignVCenter

                    Text {
                        //anchors.left: parent.left
                        Layout.alignment: Qt.AlignLeft
                        text: qsTr("35 mvg")
                        font.pixelSize: 23
                        font.bold: true
                        color: "white"
                    }

                    Text {
                        Layout.alignment: Qt.AlignLeft
                        text: qsTr("Avg. Fuel usage")
                        font.pixelSize: 15
                        color: "white"
                    }
                }
            }
        }
    }


    ColumnLayout {
        //anchors.horizontalCenter: parent.horizontalCenter
        spacing: 30
        Layout.alignment: Qt.AlignRight
        RowLayout {
            Layout.alignment: Qt.AlignHCenter
            spacing: 20
            ColumnLayout{
                Image{
                    id: roadIcon
                    height: 65
                    fillMode: Image.PreserveAspectFit
                    source: "qrc:/ui/icon/road.png"
                    sourceSize: Qt.size(45, 45)
                    smooth: true
                    visible: false

                }
                ColorOverlay {
                    anchors.fill: roadIcon
                    source: roadIcon
                    color: "gray"
                 }
            }

            ColumnLayout {
                Layout.alignment: Qt.AlignRight
                Text {
                    Layout.alignment: Qt.AlignLeft
                    text: "80 Km"
                    font.pixelSize: 23
                    font.bold: true
                    color: "white"
                }
                Text {
                    Layout.alignment: Qt.AlignLeft
                    text: qsTr("Distance")
                    font.pixelSize: 15
                    color: "white"

                }
            }
        }



        ColumnLayout {
            //anchors.horizontalCenter: parent.horizontalCenter
            Layout.alignment: Qt.AlignRight
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                spacing: 20
                ColumnLayout{
                    Image{
                        id: speedmeterIcon
                        height: 65
                        fillMode: Image.PreserveAspectFit
                        source: "qrc:/ui/icon/speedmeter.png"
                        sourceSize: Qt.size(45, 45)
                        smooth: true
                        visible: false

                    }
                    ColorOverlay {
                        anchors.fill: speedmeterIcon
                        source: speedmeterIcon
                        color: "gray"
                     }
                }

                ColumnLayout {
                    Layout.alignment: Qt.AlignRight | Qt.AlignVCenter

                    Text {
                        //anchors.left: parent.left
                        Layout.alignment: Qt.AlignLeft
                        text: qsTr("60 km/h")
                        font.pixelSize: 23
                        font.bold: true
                        color: "white"
                    }

                    Text {
                        Layout.alignment: Qt.AlignLeft
                        text: qsTr("Avg. Speed")
                        font.pixelSize: 15
                        color: "white"
                    }
                }
            }
        }
    }

 }







