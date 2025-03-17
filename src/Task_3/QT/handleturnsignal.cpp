#include "handleturnsignal.h"

HandleTurnSignal::HandleTurnSignal(QObject *parent) : QObject(parent)
{

}

bool HandleTurnSignal::turnSignal() const
{

    return m_turnSignal;
}

void HandleTurnSignal::setTurnSignal(bool turnSignal)
{
    if (m_turnSignal == turnSignal)
        return;
    m_turnSignal = turnSignal;
    emit turnSignalChanged(m_turnSignal);
}
