#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "handlepressicon.h"
#include "handleturnsignal.h"
#include "handlegetvelocitypositionvalue.h"

int main(int argc, char *argv[])
{
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    HandlePressIcon handlepressicon;
    engine.rootContext()->setContextProperty("__HandlePressIcon", &handlepressicon);
//    HandleGetVelocityPositionValue handleGetVelocityPositionValue;
//    engine.rootContext()->setContextProperty("__HandleGetVelocityPositionValue", &handleGetVelocityPositionValue);
    HandleTurnSignal handleTurnSignal;
    engine.rootContext()->setContextProperty("__HandleTurnSignal", &handleTurnSignal);

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);

    return app.exec();
}
