import QtQuick 2.11
import QtQuick.Window 2.12
import QtQuick.Controls 2.12

Item {
    CIrcularProgressBar {
                id: progress1
                lineWidth: 10
                //value: 0.8
                size: 150
                secondaryColor: "transparent"
                primaryColor: "#29b6f6"


            }
    Component.onCompleted: {
        for(var i = 0; i < 1; i+=0.1){
            progress1.value = i ;
        }
    }
}
