import QtQuick 2.12

Item {
    id: root

    property int size: 150
    property int lineWidth: 50 // Độ dày của cung tròn
    property real range: 160 // Phạm vi giá trị
    property real value: 0

    property color primaryColor: "transparent"
    property color secondaryColor: "#e0e0e0"


    width: size
    height: size

    onValueChanged: {
        // Chuyển đổi giá trị value từ phạm vi 0-150 sang 0-1
        canvas.degree = (value) * 360;
    }

    Canvas {
        id: canvas

        property real degree: 0
        property real originalAngle: -215

        anchors.fill: parent
        antialiasing: true

        onDegreeChanged: {
            requestPaint();
        }

        onPaint: {
            var ctx = getContext("2d");

            var x = root.width/2;
            var y = root.height/2;
            var gradient = ctx.createLinearGradient(0, 0, 0, height);
            gradient.addColorStop(0, "rgba(15, 255, 209, 0.6)"); // transparent at the top
            gradient.addColorStop(1, "rgba(15, 255, 209, 0.6)");

            var radius = root.size/2 - root.lineWidth / 2; // Điều chỉnh bán kính để đảm bảo độ dày
            //var radius = 0;
            var startAngle = (Math.PI/180) * (originalAngle);
            var fullAngle = (Math.PI/180) * (220 - originalAngle); // Thay đổi giới hạn
            var progressAngle = (Math.PI/180) * (originalAngle + degree);

            ctx.reset();

            ctx.lineCap = 'butt';
            ctx.lineWidth = root.lineWidth;

            ctx.beginPath();
            ctx.arc(x, y, radius, startAngle, startAngle + fullAngle);
            ctx.strokeStyle = root.secondaryColor;

            ctx.stroke();

            ctx.beginPath();
            ctx.arc(x, y, radius, startAngle, progressAngle);
            //ctx.strokeStyle = root.primaryColor;
            ctx.strokeStyle = gradient;
            ctx.stroke();
        }


    }
}
