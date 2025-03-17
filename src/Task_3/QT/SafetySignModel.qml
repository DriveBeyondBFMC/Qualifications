import QtQuick 2.0

ListModel {
    ListElement{
        name: "seatbelt"
        idName: "seatbelt"
        idCoverOverlay: "seatbeltCoverOverlay"
        sourceImage: "qrc:/ui/icon/seat.png"
        colorValue: "red"
        xCoordinates: 0
    }
    ListElement{
        name: "parkingBrake"
        idName: "parkingBrake"
        idCoverOverlay: "parkingBrakeCoverOverlay"
        sourceImage: "qrc:/ui/icon/parking-brake.png"
        colorValue: "red"
        xCoordinates: 30
    }
    ListElement{
        name: "absBrake"
        idName: "absBrake"
        idCoverOverlay: "absBrakeCoverOverlay"
        sourceImage: "qrc:/ui/icon/abs.png"
        colorValue: "gray"
        xCoordinates: 65
    }
    ListElement{
        name: "windshield"
        idName: "windshield"
        idCoverOverlay: "windshieldCoverOverlay"
        sourceImage: "qrc:/ui/icon/windshield.png"
        colorValue: "gray"
        xCoordinates: 110
    }
}
