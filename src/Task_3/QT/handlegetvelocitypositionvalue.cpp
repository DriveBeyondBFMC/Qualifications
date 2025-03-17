#include "handlegetvelocitypositionvalue.h"
#define BUFFER_SIZE 100

HandleGetVelocityPositionValue::HandleGetVelocityPositionValue(QObject *parent) : QObject(parent),
    m_Timer(new QTimer(this)),
    m_PositionValue("0"),
    m_VelocityValue("110")

{
    connect(m_Timer, &QTimer::timeout,[=](){
        emit handleDeteleFile();
    });
    m_Timer->start(10000);
}

QString HandleGetVelocityPositionValue::PositionValue() const
{
    return m_PositionValue;
}

QString HandleGetVelocityPositionValue::VelocityValue() const
{
    return m_VelocityValue;
}

void HandleGetVelocityPositionValue::setPositionValue(QString PositionValue)
{
    if (m_PositionValue == PositionValue)
        return;

    m_PositionValue = PositionValue;
    emit PositionValueChanged(m_PositionValue);
}

void HandleGetVelocityPositionValue::setVelocityValue(QString VelocityValue)
{
    if (m_VelocityValue == VelocityValue)
        return;

//    m_VelocityValue = VelocityValue;
    m_VelocityValue = "110";
    emit VelocityValueChanged(m_VelocityValue);
}

void HandleGetVelocityPositionValue::handleGetValueFromFile()
{
    /* Get path file */
    QFile file("/home/eric/data.txt");
    qDebug()  << "Open file successfully";
    /* Open text file with read mode */
    if(!file.open(QIODevice::ReadOnly| QIODevice::Text)) {
        qDebug() << "Can not open file";
        return;
    }

    /* Create lastline variable to store a data string at lastline*/
    QString lastline;

    /* Check does it has last line*/
    while(!file.atEnd()) {
        lastline = file.readLine();
    }
    /* Remove white space at first line and last line */
    lastline = lastline.trimmed();

    /* Check Is the file empty*/
    if(lastline.isEmpty()) {
        qDebug() << "The file is empty!";
        return;
    }

    /* Split data string by white space and store it into dataList.
     * The SkipEmptyParts methos will remove empty string from data string  */
    QStringList dataList = lastline.split(" ", QString::SkipEmptyParts);

    /* Check split two number from data list */
    if(dataList.size() != 2) {
        qDebug() << "Can not split two number from text line!";
    }

    /* Creating two variables to store velocity and position value */
    QString velocityValueString = dataList[0];
    QString positionValueString = dataList[1];

    /* Converting string into char type*/
    QByteArray velocityValueArray = velocityValueString.toUtf8();
    QByteArray positionValueArray = positionValueString.toUtf8();

    /* Check data size of buffer */
    if(velocityValueArray.size() >= BUFFER_SIZE || positionValueArray.size() >= BUFFER_SIZE) {
        qDebug() << "Size of number is biger size of buffer";
        return;
    }

    /* Copy data from QString into buffer */
    char velocityValueBuffer[BUFFER_SIZE];
    char positionValueBuffer[BUFFER_SIZE];

    strcpy(velocityValueBuffer, velocityValueArray.constData());
    strcpy(positionValueBuffer, positionValueArray.constData());

    /* Call setPositionValue function and send velocityValueBuffer with parameter */
    setVelocityValue(velocityValueBuffer);
    setPositionValue(positionValueBuffer);
}

void HandleGetVelocityPositionValue::handleDeteleFile()
{
    QString filePath = "/home/eric/mn/data.txt"; // Thay đổi đường dẫn tệp tương ứng
    QFile file(filePath);

    if (file.exists()) {
        if (file.remove()) {
            qDebug() << "File removed successfully!";
        } else {
            qDebug() << "Failed to remove file.";
        }
    } else {
        qDebug() << "File does not exist__.";
    }
}
