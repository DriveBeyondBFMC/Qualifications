#ifndef HANDLEGETVELOCITYPOSITIONVALUE_H
#define HANDLEGETVELOCITYPOSITIONVALUE_H

#include <QObject>
#include <QFile>
#include <QString>
#include <QDebug>
#include <QTimer>

class HandleGetVelocityPositionValue : public QObject
{
    Q_OBJECT
public:
    explicit HandleGetVelocityPositionValue(QObject *parent = nullptr);
    Q_PROPERTY(QString PositionValue READ PositionValue WRITE setPositionValue NOTIFY PositionValueChanged)
    Q_PROPERTY(QString VelocityValue READ VelocityValue WRITE setVelocityValue NOTIFY VelocityValueChanged)

    QString PositionValue() const;

    QString VelocityValue() const;

signals:

    void PositionValueChanged(QString PositionValue);
    void VelocityValueChanged(QString VelocityValue);

public slots:
    void setPositionValue(QString PositionValue);
    void setVelocityValue(QString VelocityValue);
    void handleGetValueFromFile();
    void handleDeteleFile();

private:
    QString m_PositionValue;
    QString m_VelocityValue;
    QTimer *m_Timer;
};

#endif // HANDLEGETVELOCITYPOSITIONVALUE_H
