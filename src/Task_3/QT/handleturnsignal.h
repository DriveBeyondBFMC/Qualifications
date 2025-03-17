#ifndef HANDLETURNSIGNAL_H
#define HANDLETURNSIGNAL_H

#include <QObject>
#include <QDebug>

class HandleTurnSignal : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool turnSignal READ turnSignal WRITE setTurnSignal NOTIFY turnSignalChanged)


public:
    explicit HandleTurnSignal(QObject *parent = nullptr);

    bool turnSignal() const;

signals:
    void turnSignalChanged(bool turnSignal);
    void checkState(bool value);

public slots:
    void setTurnSignal(bool turnSignal);

private:
    bool m_turnSignal;
};

#endif // HANDLETURNSIGNAL_H
