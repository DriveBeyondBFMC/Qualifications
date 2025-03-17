import QtQuick 2.0

ListModel {
    id: speficationCarModel
    ListElement{
        name: "iconRoad"
        nameCoverlay: "colorOverlayRoad"
        sourceImage: "qrc:/ui/icon/temperature.png"
        value: "78"
        desc: "Temperature"
    }

    ListElement{
        name: "iconFuel"
        nameCoverlay: "colorOverlayFuel"
        sourceImage: "qrc:/ui/icon/fuel-station.png"
        value: "34 mvg"
        desc: "Avg. Fuel usage"
    }
}
