import serial
import time
import sys
import termios
import tty
import threading

serial_port = '/dev/ttyACM0'
baud_rate = 115200

ser = serial.Serial(serial_port, baud_rate, timeout=1)

speed_param = 0
steer_param = 0
yaw_value = ""

# CMD list format
commands = {
    '0': '#kl:30;;\r\n',
    '1': '#battery:0;;\r\n',
    '2': '#instant:0;;\r\n',
    '3': '#imu:0;;\r\n',
    '4': '#resourceMonitor:0;;\r\n',
    '5': '#speed:{0};;\r\n',
    '6': '#steer:{0};;\r\n',
    '7': '#vcd:200;0;121;\r\n',
}

def send_command(command):
    ser.write(command.encode())
    print(f"Sent: {command.strip()}")

def read_response():
    global yaw_value
    i = 0
    plus = 0
    increate_flag = False
    decreate_flag = False
    previous_value = None  # Giá trị trước đó
    while True:
        if ser.in_waiting > 0:  # Kiểm tra xem có dữ liệu trong buffer hay không
            response = ser.readline().decode('utf-8').strip()
            print("\r",response)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())  # Chuyển sang chế độ raw
        ch = sys.stdin.read(1)  # Đọc một ký tự từ bàn phím
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Khôi phục cấu hình ban đầu
    return ch

def main():
    global speed_param, steer_param

    # In ra menu lựa chọn
    print("****************************")
    print("0: Enable full specification")
    print("1: Disable battery")
    print("2: Disable instant")
    print("3: Disable imu")
    print("4: Disable resource monitor")
    print("5: Control speed")
    print("6: Control steering")
    print("****************************")
    send_command(commands['0'])
    send_command(commands['1'])
    send_command(commands['2'])
    send_command(commands['4'])

    try:
        while True:
            key = get_key()  # Nhận phím từ bàn phím
            if key == 'q':  # Nếu nhấn 'q', thoát chương trình
                print("Exiting...")
                break

            elif key == '0':
                send_command(commands['0'])
                time.sleep(0.2)

            elif key == '1':
                send_command(commands['1'])
                time.sleep(0.2)

            elif key == '2':
                send_command(commands['2'])
                time.sleep(0.2)

            elif key == '3':
                send_command(commands['3'])
                time.sleep(0.2)

            elif key == '4':
                send_command(commands['4'])
                time.sleep(0.2)

            elif key == '7':
                send_command(commands['7'])
                time.sleep(0.2)

            elif key == 'w':
                speed_param = speed_param + 20
                if(speed_param > 490):
                    speed_param = 490
                send_command(commands['5'].format(speed_param))
                time.sleep(0.2)
	
            elif key == 'e':
                steer_param = 0 
                send_command(commands['6'].format(steer_param))
                time.sleep(0.2)

            elif key == 'x':
                speed_param = speed_param - 20 
                if(speed_param < -490):
                    speed_param = -490
                send_command(commands['5'].format(speed_param))
                time.sleep(0.2)

            elif key == 's':
                speed_param = 0
                send_command(commands['5'].format(speed_param))
                time.sleep(0.2)

            elif key == 'd':
                steer_param = steer_param + 25
                if(steer_param > 230):
                    steer_param = 230  
                send_command(commands['6'].format(steer_param))
                time.sleep(0.2)

            elif key == 'a':
                steer_param = steer_param - 25
                if(steer_param < -230):
                    steer_param = -230 
                send_command(commands['6'].format(steer_param))
                time.sleep(0.2)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    # Tạo một thread cho việc đọc dữ liệu từ STM32
    serial_thread = threading.Thread(target=read_response)
    serial_thread.daemon = True  # Đảm bảo rằng thread này sẽ tự động kết thúc khi chương trình chính kết thúc
    serial_thread.start()

    # Chạy chương trình chính
    main()
