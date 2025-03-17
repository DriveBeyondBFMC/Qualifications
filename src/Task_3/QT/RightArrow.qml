import QtQuick 2.0
import QtGraphicalEffects 1.0
import QtQml 2.0

Item {
    width: 100
    height: 100
    anchors.right: parent.right
//    Connections {
//           target: __HandlePressIcon
//           onSendLightSignal: {

//           }
//       }
    Image {
        id: iconRightArrow
        anchors.right: parent.right
        height: 100
        fillMode: Image.PreserveAspectFit
        source: "qrc:/ui/icon/right_arrow.png"
        sourceSize: Qt.size(85, 85)
        smooth: true
        visible: false
    }

    ColorOverlay {
        id: colorOverlayRightArrow
        anchors.fill: iconRightArrow
        source: iconRightArrow
        color: "gray"
    }

//    Timer {
//        id: colorTimer
//        interval: 800
//        repeat: true
//        running: true
//        onTriggered: {
//            if(colorOverlayRightArrow.color === Qt.rgba(0, 0, 0, 1)) {
//                colorOverlayRightArrow.color = Qt.rgba(0, 1, 0, 1);
//            } else {
//                colorOverlayRightArrow.color = Qt.rgba(0, 0, 0, 1);
//            }
//        }
//    }

}
