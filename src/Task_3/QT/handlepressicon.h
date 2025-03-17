#ifndef HANDLEPRESSICON_H
#define HANDLEPRESSICON_H
#include <QDebug>
#include <QProcess>
#include <QFile>
#include <QTextStream>
#include <QObject>
#include <QColor>
#include <QDebug>
#include <QColor>
#include <QString>
#include <QProcess>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <QFile>
#include <QTimer>
class HandlePressIcon : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString speedValue READ speedValue WRITE setSpeedValue NOTIFY speedValueChanged)
    Q_PROPERTY(QString positionValue READ positionValue WRITE setPositionValue NOTIFY positionValueChanged)

public:
    explicit HandlePressIcon(QObject *parent = nullptr);
    QString speedValue() const;
    QString positionValue() const;

signals:
    void sendLightSignal(QColor colorParam, int idColorName);
    void speedValueChanged(QString speedValue);
    void positionValueChanged(QString positionValue);

public slots:
    void handlePressIcon(int id);
    void get_speed_value();
    void handle_delete_file();
    void setSpeedValue(QString speedValue);
    void setPositionValue(QString positionValue);

private:
    QString m_speedValue;
    QString m_positionValue;
};

#endif // HANDLEPRESSICON_H
