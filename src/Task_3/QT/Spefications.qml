import QtQuick 2.0
import QtQuick.Layouts 1.12
import QtQuick.Controls 2.12
import QtGraphicalEffects 1.0


Item {
    property int valueBattery: 83
    function changeColorFollowValue(){

        if(valueBattery < 10){
            return "red"
        }
        else if(valueBattery >= 10 && valueBattery < 80){
            return "#FA8200"
        }
        else if(valueBattery >= 80 && valueBattery <= 100){
            return "#43ED10"
        }
    }

    Column{
        spacing: 40
        RowLayout{
            ColumnLayout{
                spacing: 50
                Rectangle {
                    height: 80;
                    width: 200;
                    color: "transparent";
                    border.width: 0
                    border.color: "white"
                    radius: 5

                    ColumnLayout{
                        anchors.centerIn: parent
                        RowLayout{

                            Image{
                                id: iconChargingFollow
                                anchors{
                                    left:parent.left
                                    verticalCenter: parent.verticalCenter
                                }
                                height: 70
                                fillMode: Image.PreserveAspectFit
                                source: "qrc:/ui/icon/charging(2).png"
                                sourceSize: Qt.size(50, 50)
                                smooth: true
                                visible: false
                            }
                            ColorOverlay {
                                    id: colorOverlayCharging
                                    anchors.fill: iconChargingFollow
                                    source: iconChargingFollow
                                    color: changeColorFollowValue()
                                    //color: "red"
                            }
                            Text{
                                id: textShowBattery
                                text: valueBattery + "%"
                                //color: "#1FCD9C"
                                color: "white"
                                anchors.verticalCenter: parent.verticalCenter
                                anchors{
                                    right: parent.right
                                }
                                font.pointSize: 16
                            }
                        }
                        ProgressBar {
                            id: control
                            value: valueBattery / 100
                            padding: 2
                            background: Rectangle {
                                implicitWidth: 200
                                implicitHeight: 6
                                color: "transparent"
                                radius: 3
                            }

                            contentItem: Item {
                                implicitWidth: 100
                                implicitHeight: 5

                                Rectangle {
                                    width: control.visualPosition * parent.width
                                    height: parent.height
                                    radius: 2
                                    //color: "#1FCD9C"
                                    color:  changeColorFollowValue()
                                }
                            }
                        }
                    }

                }
            }

            ColumnLayout{
                spacing: 20
                Rectangle {
                    height: 80;
                    width: 200;
                    color: "transparent";
                    border.width: 0
                    border.color: "white"
                    radius: 5

                    ColumnLayout{
                        anchors.centerIn: parent
                        RowLayout{
                            Image{
                                id: iconPluginsFollow
                                anchors{
                                    left:parent.left
                                    verticalCenter: parent.verticalCenter
                                }
                                height: 70
                                fillMode: Image.PreserveAspectFit
                                source: "qrc:/ui/icon/charging-station.png"
                                sourceSize: Qt.size(50, 50)
                                smooth: true
                                visible: false
                            }
                            ColorOverlay {
                                    id: colorOverlayPlugin
                                    anchors.fill: iconPluginsFollow
                                    source: iconPluginsFollow
    //                                color: changeColorFollowValue()
                                    //color: "red"
                                    color: "#12A6EB"
                            }
                            Text{
                                text: "70Km"
    //                            color: "#FA8200"
                                color: "white"
                                anchors.verticalCenter: parent.verticalCenter
                                anchors{
                                    right: parent.right
                                }
                                font.pointSize: 16
                            }


                        }


                        ProgressBar {
                            id: controlPlugin
                            value: 0.5
                            padding: 2

                            background: Rectangle {
                                implicitWidth: 200
                                implicitHeight: 6
                                color: "transparent"
                                radius: 3
                            }

                            contentItem: Item {
                                implicitWidth: 200
                                implicitHeight: 5

                                Rectangle {
                                    width: control.visualPosition * parent.width
                                    height: parent.height
                                    radius: 2
                                    color: "#12A6EB"
                                }
                            }
                        }
                    }

                }
            }

        }



        RowLayout{
            ColumnLayout{
                spacing: 20
                Rectangle {
                    height: 80;
                    width: 200;
                    color: "transparent";
                    border.width: 0
                    border.color: "white"
                    radius: 5
                    ColumnLayout{
                        anchors.centerIn: parent
                        RowLayout{
                            Image{
                                id: iconPluginsFollow1
                                anchors{
                                    left:parent.left
                                    verticalCenter: parent.verticalCenter
                                }
                                height: 70
                                fillMode: Image.PreserveAspectFit
                                source: "qrc:/ui/icon/battery_status.png"
                                sourceSize: Qt.size(50, 50)
                                smooth: true
                                visible: false
                            }
                            ColorOverlay {
                                    id: colorOverlayPlugin1
                                    anchors.fill: iconPluginsFollow1
                                    source: iconPluginsFollow1
    //                                color: changeColorFollowValue()
                                    //color: "red"
                                    color: "#FA8200"
                            }
                            Text{
                                text: "70Km"
    //                            color: "#FA8200"
                                color: "white"
                                anchors.verticalCenter: parent.verticalCenter
                                anchors{
                                    right: parent.right
                                }
                                font.pointSize: 16
                            }
                        }
                        ProgressBar {
                            id: controlPlugin1
                            value: 0.5
                            padding: 2

                            background: Rectangle {
                                implicitWidth: 200
                                implicitHeight: 6
                                color: "transparent"
                                radius: 3
                            }

                            contentItem: Item {
                                implicitWidth: 200
                                implicitHeight: 5

                                Rectangle {
                                    width: control.visualPosition * parent.width
                                    height: parent.height
                                    radius: 2
                                    color: "#FA8200"
                                }
                            }
                        }
                    }

                }
            }

        }
    }


}

