import sys

# Read original F1 source as GBK, write as UTF-8
src = r"c:\Users\28935\Desktop\Test_2\F1-高顺低逆-减速比60-FLASH\F1-高顺低逆-减速比60-FLASH\HARDWARE\STEPPER_MOTOR\step_motor.c"
dst = r"c:\Users\28935\Desktop\Test_2\new\Core\Src\step_motor_utf8.c"

with open(src, 'rb') as f:
    data = f.read()

text = data.decode('gbk')

with open(dst, 'w', encoding='utf-8') as f:
    f.write(text)

print("Converted OK, lines:", text.count('\n'))
