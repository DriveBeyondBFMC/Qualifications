import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.0
import QtQuick.Controls 2.12
import QtQuick.Shapes 1.12

Rectangle {
    id: dashboardSimluation

    width: 1280
    height: 800
    anchors.centerIn: parent
    border.width: 13
    border.color: "#29b6f6"
    color: "transparent"
    radius: 250


    // Function to create a filled trapezoid in the canvas with a gradient
    function drawGradientTrapezoid(ctx, x, y, width, height, gradient) {
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + width, y);
        ctx.lineTo(x + width * 0.90, y + height);
        ctx.lineTo(x + width * 0.10, y + height);
        ctx.closePath();

        // Use the gradient as the fill style
        ctx.fillStyle = gradient;
        ctx.fill();
    }

    Canvas {
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        width: 730
        height: 60
        rotation: 180

        onPaint: {
            var ctx = getContext("2d");
            var gradient = ctx.createLinearGradient(0, 0, 0, height);
            gradient.addColorStop(0, "rgba(41, 181, 245, 1)");
            gradient.addColorStop(1, "rgba(41, 181, 245, 1)");

            // Draw a filled trapezoid with the gradient
            drawGradientTrapezoid(ctx, 0.5, 0, width, height, gradient);
        }
    }

    Canvas {
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        width: 730
        height: 80
        onPaint: {
            var ctx = getContext("2d");

            // Verify the rendering context
            if (ctx) {
                // Create a linear gradient
                var gradient = ctx.createLinearGradient(0, 0, 0, height);
                gradient.addColorStop(0, "rgba(41, 181, 245, 1)");
                gradient.addColorStop(1, "rgba(41, 181, 245, 0)");

                // Draw a filled trapezoid with the gradient
                drawGradientTrapezoid(ctx, 0.5, 0, width, height, gradient);
            } else {
                console.error("Failed to get 2D rendering context.");
            }
        }
    }


    RowLayout {
        spacing: 20
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter

        ColumnLayout {
            id: dashboardRow
            spacing: 30
            anchors.verticalCenter: parent.verticalCenter

            LightShowConfiguration {
                x: 28
                y: 25
            }

            RowLayout {
                anchors.centerIn: parent
                LeftArrow {}
                SafetySignConfiguration {
                    x: 28
                    y: 440
                }
                CircularGaugeISpeedMeter {
                    id: circularGaugeISpeedMeter
                }
                RightArrow {}
            }
        }

        ColumnLayout {

            anchors.right: parent.right
            anchors.bottom: parent.bottom
            spacing: 30
            SpeficationCarConfiguration{
            }
            CIrcularProgressBarBattery {

                size: 350
                lineWidth: 12
                value: 0.8
                Layout.alignment: Qt.AlignBottom

                ColumnLayout {
                    anchors.centerIn: parent

                    Text {
                        text: "75%";
                        color: "white"
                        font.pixelSize: 75
                        font.bold: true
                    }

                    Text {
                        //Layout.alignment: Qt.AlignHCenter
                        anchors.horizontalCenter: parent.horizontalCenter
                        text: "Battery Charge"
                        font.pixelSize: 25
                        color: "#15FFD1"
                    }
                }
            }


        }
    }
}
