#include "handlepressicon.h"
#include "handleturnsignal.h"
#define BUFFER_SIZE 50

HandleTurnSignal handleTurnSignal;

HandlePressIcon::HandlePressIcon(QObject *parent) : QObject(parent),
    m_speedValue("0"),
    m_positionValue("0")
{
}

void HandlePressIcon::handlePressIcon(int id)
{
    QColor m_color;
    static bool isPress = false;

    auto toggleColor = [&]() {
        if (!isPress) {
            isPress = true;
            if (id == 1 || id == 2 || id == 3 || id == 4) {
                m_color = "#43ED10";

            } else {
                m_color = "yellow";
            }
        } else {
            isPress = false;

            m_color = "gray";
        }
    };
    handleTurnSignal.setTurnSignal(true);
    toggleColor();
    emit sendLightSignal(m_color, id);

    QProcess process;
    QString command = "echo 1 > /dev/etx_device";
    process.start(command);
    process.waitForFinished();
    QString output = process.readAllStandardOutput();


}

void HandlePressIcon::get_speed_value()
{
    QFile file("/home/eric/data.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        //qDebug() << "Lỗi mở tập tin: " << file.errorString();
        return;
    }

    QString lastLine;
    while (!file.atEnd()) {
        lastLine = file.readLine();
    }

    lastLine = lastLine.trimmed(); // Loại bỏ khoảng trắng ở đầu và cuối dòng

    if (lastLine.isEmpty()) {
        //qDebug() << "Tập tin trống.";
        return;
    }

    QStringList numbers = lastLine.split(" ", QString::SkipEmptyParts);
    if (numbers.size() != 2) {
        //qDebug() << "Không thể tách 2 số từ dòng: " << lastLine;
        return;
    }

    QString firstNumberStr = numbers[0];
    QString secondNumberStr = numbers[1];

    // Chuyển đổi từ QString sang buffer char *
    QByteArray firstNumberByteArray = firstNumberStr.toUtf8();
    QByteArray secondNumberByteArray = secondNumberStr.toUtf8();

    // Kiểm tra kích thước của buffer
    if (firstNumberByteArray.size() >= BUFFER_SIZE || secondNumberByteArray.size() >= BUFFER_SIZE) {
        //qDebug() << "Kích thước số lớn hơn kích thước buffer.";
        return;
    }

    // Sao chép dữ liệu từ QString sang buffer
    char firstNumberBuffer[BUFFER_SIZE];
    char secondNumberBuffer[BUFFER_SIZE];

    strcpy(firstNumberBuffer, firstNumberByteArray.constData());
    strcpy(secondNumberBuffer, secondNumberByteArray.constData());



    setSpeedValue(firstNumberBuffer);
    setPositionValue(secondNumberBuffer);
}

void HandlePressIcon::handle_delete_file()
{
    QString filePath = "/home/eric/mn/data.txt"; // Thay đổi đường dẫn tệp tương ứng
    QFile file(filePath);

    if (file.exists()) { // Kiểm tra xem tệp có tồn tại không
        if (file.remove()) { // Xóa tệp
            //qDebug() << "File removed successfully!";
        } else {
            //qDebug() << "Failed to remove file.";
        }
    } else {
        //qDebug() << "File does not exist.";
    }
}

void HandlePressIcon::setSpeedValue(QString speedValue)
{


    if (m_speedValue == speedValue)
        return;
    m_speedValue = speedValue;
//    m_speedValue = "300";
    emit speedValueChanged(m_speedValue);

}

void HandlePressIcon::setPositionValue(QString positionValue)
{
    if (m_positionValue == positionValue)
        return;

    m_positionValue = positionValue;
    emit positionValueChanged(m_positionValue);
}

QString HandlePressIcon::speedValue() const
{

    return m_speedValue;
}

QString HandlePressIcon::positionValue() const
{
    return m_positionValue;
}




