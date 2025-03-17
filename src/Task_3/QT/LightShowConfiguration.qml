import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.12
import QtGraphicalEffects 1.0
Item {
    property bool flag: true;
    Connections{
        target: __HandlePressIcon
        onSendLightSignal: {
           if(idColorName == 1) {
               packingLightCoverOverlay.color = colorParam;
           }
           if(idColorName == 4) {
               brightnessCoverOverlay.color = colorParam;
               headlight1CoverOverlay.color = colorParam;
               headlightCoverOverlay.color = "gray";
               flag = false;
           }
           if(idColorName == 2 ) {
               headlightCoverOverlay.color = "gray";
               headlight1CoverOverlay.color = colorParam;
           }
           if(idColorName == 3 && flag == false) {
               flag = true;
               headlightCoverOverlay.color = "#43ED10";
               headlight1CoverOverlay.color = "gray";
           }
        }

    }
    Column{
        id: settingPannel
        width: parent.width
        anchors.centerIn: parent
        spacing: 10
        //        Repeater{
        //            id: parameterRepeater
        //            model: lightShowModel

        ColumnLayout{
            Image{
                id: packingLight
                x: 110-10
                y: 30
                height: 55
                fillMode: Image.PreserveAspectFit
                source: "qrc:/ui/icon/parking-lights.png"
                sourceSize: Qt.size(38, 38)
                smooth: true
                visible: false

            }
            ColorOverlay {
                id: packingLightCoverOverlay
                anchors.fill: packingLight
                source: packingLight
                //color: "#43ED10"
                color: "gray"
                //visible: false
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        __HandlePressIcon.handlePressIcon(1);
                    }
                }

            }
        }




        ColumnLayout{
            Image{
                id: headlight1
                x: 65-10
                y: 30
                height: 55
                fillMode: Image.PreserveAspectFit
                source: "qrc:/ui/icon/headlight1.png"
                sourceSize: Qt.size(38, 38)
                smooth: true
                visible: false

            }
            ColorOverlay {
                id: headlight1CoverOverlay
                anchors.fill: headlight1
                source: headlight1
                //color: "#43ED10"
                color: "gray"
                //visible: false
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        __HandlePressIcon.handlePressIcon(2);
                    }
                }

            }
        }



        ColumnLayout{
            Image{
                id: headlight
                x: 30-10
                y: 30
                height: 55
                fillMode: Image.PreserveAspectFit
                source: "qrc:/ui/icon/headlight.png"
                sourceSize: Qt.size(38, 38)
                smooth: true
                visible: false

            }
            ColorOverlay {
                id: headlightCoverOverlay
                anchors.fill: headlight
                source: headlight
                //color: "#43ED10"
                color: "gray"
                //visible: false
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        __HandlePressIcon.handlePressIcon(3);
                    }
                }

            }
        }




        ColumnLayout{
            Image{
                id: brightness
                x: 0-10
                y: 30
                height: 55
                fillMode: Image.PreserveAspectFit
                source: "qrc:/ui/icon/sun.png"
                sourceSize: Qt.size(38, 38)
                smooth: true
                visible: false

            }
            ColorOverlay {
                id: brightnessCoverOverlay
                anchors.fill: brightness
                source: brightness
                //color: "#43ED10"
                color: "gray"
                //visible: false
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        __HandlePressIcon.handlePressIcon(4);
                    }
                }

            }
        }


    }
}
