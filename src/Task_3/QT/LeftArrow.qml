import QtQuick 2.0
import QtGraphicalEffects 1.0
Item{
    width: 100
    height: 100
    anchors{
        left: parent.left
    }

    Image{
        id: iconLeftArrow
        anchors{
            left:parent.left
        }
        height: 100
        fillMode: Image.PreserveAspectFit
        source: "qrc:/ui/icon/left_arrow.png"
        sourceSize: Qt.size(85, 85)
        smooth: true
        visible: false

    }
    ColorOverlay {
        id: colorOverlayLeftArrow
        anchors.fill: iconLeftArrow
        source: iconLeftArrow
        //color: "#43ED10"
        color: "#727171"
        //visible: false
      }
//    Timer {
//        id: colorTimer
//        interval: 800
//        repeat: true
//        running: true
//        onTriggered: {
//            if(colorOverlayLeftArrow.color === Qt.rgba(0, 0, 0, 1)) {
//                colorOverlayLeftArrow.color = Qt.rgba(0, 1, 0, 1);
//            } else {
//                colorOverlayLeftArrow.color = Qt.rgba(0, 0, 0, 1);
//            }
//        }
//    }
}
