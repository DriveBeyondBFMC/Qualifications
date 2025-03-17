import QtQuick 2.0
import QtQuick.Controls 1.4
Item {
    id: rootComp
    VehicleConfigModel{
        id: vehicleConfigModel
    }
    Column{
        id: settingPannel
        width: parent.width
        anchors.centerIn: parent
        spacing: 10
        Repeater{
            id: parameterRepeater
            model: vehicleConfigModel
                Row{
                    id: name
                    spacing: 10
                    Text{
                        id: rpmLabel
                        anchors.verticalCenter: parent.verticalCenter
                        width: settingPannel.width * 0.4
                        font.pointSize: 15
                        text: displayText
                        color: "white"
                    }
                    TextField{

                        anchors.verticalCenter: parent.verticalCenter
                        width: settingPannel.width * 0.4
                        font.pointSize: 15
                        text: value
                    }
                    Text{

                        anchors.verticalCenter: parent.verticalCenter
                        width: settingPannel.width * 0.4
                        font.pointSize: 15
                        text: unit
                        color: "white"

                    }
                }
        }

    }
}
