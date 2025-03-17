import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4
import QtQuick.Extras.Private 1.0
import QtQuick.Shapes 1.12
import QtQuick.Layouts 1.12
import QtGraphicalEffects 1.12
//Item {

RowLayout{

    Rectangle{
        width: 530
        height: 530
        color: Qt.rgba(52/255, 58/255, 64/255, 0.7)

        radius: width/2
        border.width: 6
        border.color: "#15ffd1"

        CircularGauge{
            id: speedmeter
            width: 470
            height: 470
            maximumValue: 220
            anchors.centerIn: parent
            value:  __HandlePressIcon.speedValue
            Behavior on value {
                NumberAnimation {
                    duration: 250
                }
            }
            style: CircularGaugeStyle{

                labelStepSize: 20
                minimumValueAngle: -140
                maximumValueAngle: 140

                function changeValue() {
                    if (parseInt(speedmeter.value) > 0 && parseInt(speedmeter.value) < 220) {
                        return '#29b6f6';
                    } else if (parseInt(speedmeter.value) >= 100) {
                        return 'red';
                    }
                }

                needle: Rectangle {
                    y: outerRadius * 0.22
                    implicitWidth: outerRadius * 0.03
                    implicitHeight: outerRadius * 0.9
                    antialiasing: true
                    color: "#e5e5e5"
                }
                foreground: Item {
                    //visible: false
                    CIrcularProgressBar {
                        anchors.centerIn: parent
                        id: circularProgressBar
                        lineWidth: 50
                        value: __HandlePressIcon.speedValue / 300;
//                        value: 100/230
                        size: 320
                        z: 30
                        secondaryColor: "transparent"

                        Behavior on value {
                            NumberAnimation {
                                duration: 250

                            }
                        }
                        primaryColor: "#29b5f580"
                        Rectangle {
                            width: 230
                            height: 230
                            z: 10
                            radius: width / 2
                            color: "#343a40"
                            border.color: "white"
                            border.width: 10
                            anchors.centerIn: parent

                            Rectangle{
                                anchors.centerIn: parent
                                width: 130
                                height: 110
                                radius: 10
                                color: "transparent"
                                Text{
                                    id: speedValue
                                    y: 10
                                    anchors.horizontalCenter: parent.horizontalCenter
                                    text: __HandlePressIcon.speedValue
                                    //color: changeValue()

                                    color: "white";
                                    font.bold: true
                                    font.pixelSize: 65


                                    Text{
                                        y: 74
                                        anchors.horizontalCenter: parent.horizontalCenter
                                        text: "mm/s"
                                        color: "#15FFD1"
                                        font.pixelSize: 30
                                        font.bold: true
                                    }
                                }
                            }
                        }
                    }
                }

                function degreesToRadians(degrees) {
                    return degrees * (Math.PI / 220);
                }

                background: Canvas {
                    onPaint: {
                        var ctx = getContext("2d");
                        ctx.reset();

                        // Tạo gradient từ màu đỏ (#CF2626) đến màu vàng (#FFFF00)
                        var gradient = ctx.createLinearGradient(0, 0, outerRadius * 2, 0);
                        //gradient.addColorStop(0, "#CF2626");
                        //                        29B5F5
                        gradient.addColorStop(0, "white");
                        //                        gradient.addColorStop(1, "#FFFF00");
                        gradient.addColorStop(1, "#CF2626");

                        ctx.beginPath();
                        ctx.strokeStyle = gradient;
                        ctx.lineWidth = outerRadius * 0.02;

                        ctx.arc(outerRadius, outerRadius, outerRadius - ctx.lineWidth / 2,
                                degreesToRadians(valueToAngle(-40) - 90), degreesToRadians(valueToAngle(228) - 90));
                        ctx.stroke();
                    }
                }


                tickmark: Rectangle {
                    visible: styleData.value <0 || styleData.value % 10 == 0
                    implicitWidth: outerRadius * 0.02
                    antialiasing: true
                    implicitHeight: outerRadius * 0.06
                    color: styleData.value >= 180 ? "#CF2626" : "#e5e5e5"
                }
                minorTickmark: Rectangle {
                    visible: styleData.value < 0
                    implicitWidth: outerRadius * 0.01
                    antialiasing: true
                    implicitHeight: outerRadius * 0.03
                    color: "red"
                }
                tickmarkLabel:  Text {
                    font.pixelSize: Math.max(6, outerRadius * 0.1)
                    text: styleData.value
                    color: styleData.value >= 180 ? "#CF2626" : "#e5e5e5"
                    //color: "white"
                    antialiasing: true
                }


            }
            Behavior on value{
                NumberAnimation{
                    duration: 3000
                }
            }

            Rectangle{
                anchors.horizontalCenter: parent.horizontalCenter
                y: 380
                width: 150
                height: 90
                radius: 10
                color: "#adb5bd"
                Timer{
                    interval: 500
                    running: true
                    repeat: true
                    onTriggered: {
                        __HandlePressIcon.get_speed_value();
                    }
                }

                Timer{
                    interval: 10000
                    running: true
                    repeat: true
                    onTriggered: {
                        __HandlePressIcon.handle_delete_file();
                    }
                }
                Text{
                    y: 5
                    anchors.horizontalCenter: parent.horizontalCenter
//                    text: __HandlePressIcon.positionValue
                    text: "bfmc"
                    //color: "#0077b6"
                    color: "white"
                    font.bold: true
                    font.pixelSize: 35

                    Text{
                        y: 53
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: "drivebeyond"
                        color: "white"
                        font.pixelSize: 17
                        //color: "#15FFD1"
                    }
                }
            }
        }

    }

}



