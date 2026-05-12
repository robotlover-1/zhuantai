import re

with open(r'c:\Users\28935\Desktop\test\Core\Src\step_motor.c', 'r', encoding='utf-8') as f:
    content = f.read()

original = content
counts = {}

# 1. Replace alarm pulse (GPIO_PIN_SET/GPIO_PIN_RESET variant)
pattern_alarm_set = r'HAL_GPIO_WritePin\(ST1_ALARM_GPIO_PORT,\s*ST1_ALARM_GPIO_PIN,\s*GPIO_PIN_SET\);\s*\n\s*delay_ms\(50\);\s*\n\s*HAL_GPIO_WritePin\(ST1_ALARM_GPIO_PORT,\s*ST1_ALARM_GPIO_PIN,\s*GPIO_PIN_RESET\);'
counts['alarm_pulse'] = len(re.findall(pattern_alarm_set, content))
content = re.sub(pattern_alarm_set, 'alarm_pulse();', content)

# 2. Replace done pulse (GPIO_PIN_SET/GPIO_PIN_RESET variant, 100ms)
pattern_done_set = r'HAL_GPIO_WritePin\(ST1_DONE_GPIO_PORT,\s*ST1_DONE_GPIO_PIN,\s*GPIO_PIN_SET\);\s*\n\s*delay_ms\(100\);\s*\n\s*HAL_GPIO_WritePin\(ST1_DONE_GPIO_PORT,\s*ST1_DONE_GPIO_PIN,\s*GPIO_PIN_RESET\);'
counts['done_pulse'] = len(re.findall(pattern_done_set, content))
content = re.sub(pattern_done_set, 'done_pulse();', content)

# 3. Replace done pulse (1/0 variant, 50ms)
pattern_done_1 = r'HAL_GPIO_WritePin\(ST1_DONE_GPIO_PORT,\s*ST1_DONE_GPIO_PIN,\s*1\);\s*\n\s*delay_ms\(50\);\s*\n\s*HAL_GPIO_WritePin\(ST1_DONE_GPIO_PORT,\s*ST1_DONE_GPIO_PIN,\s*0\);'
c1 = len(re.findall(pattern_done_1, content))
content = re.sub(pattern_done_1, 'done_pulse();', content)
counts['done_pulse_50ms'] = c1

# 4. Replace emergency stop (full 6-line block)
pattern_stop = r'dcmotor_stop\(\);\s*\n\s*duzhuan_flag\s*=\s*0;\s*\n\s*motor_pwm_set\(0\);\s*\n\s*run_flag\s*=\s*0;\s*\n\s*run_printf_flag\s*=\s*0;\s*\n\s*HAL_TIM_Base_Stop_IT\(&TIM5_Handler\);'
counts['motor_safe_stop'] = len(re.findall(pattern_stop, content))
content = re.sub(pattern_stop, 'motor_safe_stop();', content)

# 5. Replace emergency stop with error set (7-line variant)
pattern_stop_err = r'dcmotor_stop\(\);\s*\n\s*duzhuan_flag\s*=\s*0;\s*\n\s*motor_pwm_set\(0\);\s*\n\s*run_flag\s*=\s*0;\s*\n\s*run_printf_flag\s*=\s*0;\s*\n\s*HAL_TIM_Base_Stop_IT\(&TIM5_Handler\);\s*\n\s*error\s*=\s*1;'
counts['motor_safe_stop_err'] = len(re.findall(pattern_stop_err, content))
content = re.sub(pattern_stop_err, 'motor_safe_stop();\nerror = 1;', content)

if content != original:
    with open(r'c:\Users\28935\Desktop\test\Core\Src\step_motor.c', 'w', encoding='utf-8') as f:
        f.write(content)
    print("Replacements done:")
    for k, v in counts.items():
        if v > 0:
            print(f"  {k}: {v}")
else:
    print("No replacements made")

# Also process cmd_handler.c
with open(r'c:\Users\28935\Desktop\test\Core\Src\cmd_handler.c', 'r', encoding='utf-8') as f:
    content2 = f.read()

original2 = content2
for name, pattern in [
    ('alarm_pulse', r'HAL_GPIO_WritePin\(ST1_ALARM_GPIO_PORT,\s*ST1_ALARM_GPIO_PIN,\s*GPIO_PIN_SET\);\s*\n\s*delay_ms\(50\);\s*\n\s*HAL_GPIO_WritePin\(ST1_ALARM_GPIO_PORT,\s*ST1_ALARM_GPIO_PIN,\s*GPIO_PIN_RESET\);'),
]:
    c = len(re.findall(pattern, content2))
    content2 = re.sub(pattern, name + '();', content2)
    if c > 0:
        print(f"  cmd_handler.c {name}: {c}")

if content2 != original2:
    with open(r'c:\Users\28935\Desktop\test\Core\Src\cmd_handler.c', 'w', encoding='utf-8') as f:
        f.write(content2)
