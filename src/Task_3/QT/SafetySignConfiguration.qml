import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.12
import QtGraphicalEffects 1.0
Item {
    id: rootComp
    SafetySignModel{
        id: safetySignModel
    }
    Column{
        id: settingPannel
        width: parent.width
        anchors.centerIn: parent
        spacing: 10
        Repeater{
            id: parameterRepeater
            model: safetySignModel
            ColumnLayout{
                Image{
                    id: idName
                    x: xCoordinates-10
                    y: 30
                    height: 55
                    fillMode: Image.PreserveAspectFit
                    source: sourceImage
                    sourceSize: Qt.size(38, 38)
                    smooth: true
                    visible: false

                }
                ColorOverlay {
                    id: idCoverOverlay
                    anchors.fill: idName
                    source: idName
                    //color: "#43ED10"
                    color: colorValue
                    //visible: false

                }
            }
        }

    }
}
